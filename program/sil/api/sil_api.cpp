// SIL の C ABI 実体
//
// 責務:
//   1) 実機 main() の初期化シーケンスを再現する
//   2) 400Hz tick を駆動し、core1(100Hz) の完了まで見届ける
//   3) duty と内部状態を Python へ渡す
//
// 実機 pico_copter.cpp は main() を持つためリンクしない。
// そこで定義されていたグローバルはここで定義する。
#include "sil_api.h"

#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"

#include <chrono>
#include <cstdio>
#include <cstring>

// --- 実機 pico_copter.cpp が持っていたグローバル --------------------------
uint8_t Arm_flag = 0;
uint8_t Red_flag = 0;
semaphore_t sem;

// --- ファーム側の関数（C++ リンケージ。extern "C" にしてはいけない） -----
void pwm_pins_early_init(void);
void pwm_init(void);
void radio_init(void);
void variable_init(void);
void control_init(void);
void imu_mag_init(void);
void loop_400Hz(void);
void angle_control(void);
void rgbled_off(void);

// --- SIL 側が提供する注入 API（これらは C リンケージ） --------------------
extern "C" {
void sil_sensor_set_imu(const float[3], const float[3], const float[3]);
void sil_sensor_set_range_mm(uint16_t);
}
extern uint16_t Chdata[18];
extern uint8_t  ESC_calib;
extern uint8_t  Flight_mode;
extern uint8_t  Logflag;
extern float Phi, Theta, Psi;
extern float Phi_ref, Theta_ref, Psi_ref;
extern float Pref, Qref, Rref;
extern float P_com, Q_com, R_com;
extern float T_ref, T_stick, altitude;
extern float FR_duty, FL_duty, MR_duty, ML_duty, RR_duty, RL_duty;
extern float Phi_bias, Theta_bias, Psi_bias;
extern float Pbias, Qbias, Rbias;
extern float Wp, Wq, Wr;
extern float Ax, Ay, Az, Mx, My, Mz;
extern float MN, ME, MD;
#include <Eigen/Dense>
extern Eigen::Matrix<float, 7, 1> Xe;
#include "pid.hpp"
extern PID phi_pid, theta_pid, psi_pid, p_pid, q_pid, r_pid;

// --- PWM スライス割り当て -------------------------------------------------
// ⚠ pwm.cpp のスライス名は実際のモータ位置と一致していない。
//    Slice_num_front(7) = FR/RR, Slice_num_middle(2) = FL/RL, Slice_num_rear(1) = MR/ML
namespace {
constexpr uint kSliceFront  = 7;  // A=FR, B=RR
constexpr uint kSliceMiddle = 2;  // A=FL, B=RL
constexpr uint kSliceRear   = 1;  // A=MR, B=ML

// pwm.hpp: duty = (DUTYMAX-DUTYMIN)*d + DUTYMIN, DUTYMIN=1375, DUTYMAX=2375
constexpr float kDutyMin = 1375.0f;
constexpr float kDutySpan = 1000.0f;

inline float level_to_duty(uint16_t level) {
    return ((float)level - kDutyMin) / kDutySpan;
}

irq_handler_t g_loop400 = nullptr;
uint32_t g_tick = 0;
uint64_t g_last_loop_ns = 0;
uint64_t g_last_core1_ns = 0;
bool g_inited = false;
}

// =========================================================================
// ライフサイクル
// =========================================================================
extern "C" void sil_set_stdout(int enabled) {
    // freopen は fd 1 ごと差し替えてしまい、ホスト(Python)の出力まで消える。
    // macOS/glibc とも stdout は代入可能な FILE* なので、ポインタだけ退避・差替する。
    static FILE *real_stdout = nullptr;
    static FILE *devnull = nullptr;
    if (real_stdout == nullptr) real_stdout = stdout;
    if (enabled) {
        stdout = real_stdout;
    } else {
        if (devnull == nullptr) devnull = std::fopen("/dev/null", "w");
        if (devnull != nullptr) stdout = devnull;
    }
}

extern "C" int sil_init(void) {
    if (g_inited) return 0;
    sil_set_stdout(0);          // 既定でファームの printf を捨てる

    // 実機 main() と同じ順序
    pwm_pins_early_init();
    imu_mag_init();                         // sensor_sil.cpp が提供
    radio_init();
    variable_init();
    control_init();
    rgbled_off();

    ESC_calib = 0;
    pwm_init();                             // ここで IRQ ハンドラが登録される

    g_loop400 = sil_irq_get_handler(PWM_IRQ_WRAP);
    if (g_loop400 == nullptr) return -1;

    sem_init(&sem, 0, 1);
    multicore_launch_core1(angle_control);

    Arm_flag = 1;
    g_inited = true;
    return 0;
}

extern "C" void sil_shutdown(void) { g_inited = false; }

// =========================================================================
// 入力注入
// =========================================================================
extern "C" void sil_set_imu(const float acc[3], const float gyr[3], const float mag[3]) {
    sil_sensor_set_imu(acc, gyr, mag);
}
extern "C" void sil_set_range_mm(uint16_t mm) { sil_sensor_set_range_mm(mm); }
extern "C" void sil_set_rc(const uint16_t ch[18]) { std::memcpy(Chdata, ch, sizeof(uint16_t) * 18); }
extern "C" void sil_set_arm_flag(uint8_t v) { Arm_flag = v; }
extern "C" void sil_set_flight_mode(uint8_t mode) { Flight_mode = mode; }

extern "C" void sil_set_pid(int which, float kp, float ti, float td, float eta, float h) {
    PID *tbl[6] = { &phi_pid, &theta_pid, &psi_pid, &p_pid, &q_pid, &r_pid };
    if (which < 0 || which > 5) return;
    const float step = (h > 0.0f) ? h : ((which <= 2) ? 0.01f : 0.0025f);
    tbl[which]->set_parameter(kp, ti, td, eta, step);
    tbl[which]->reset();
}

// =========================================================================
// 実行
// =========================================================================
extern "C" void sil_tick_400hz(void) {
    if (!g_inited || g_loop400 == nullptr) return;

    // 実機 PWM 周期 400Hz = 2500us
    sil_time_advance_us(2500);

    uint64_t rel_before = 0, acq_before = 0;
    sil_sched_snapshot(&rel_before, &acq_before);

    auto t0 = std::chrono::steady_clock::now();
    g_loop400();                 // = loop_400Hz（core0 側）
    auto t1 = std::chrono::steady_clock::now();
    g_last_loop_ns = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(t1 - t0).count();

    // core1 が起動されたなら、その 100Hz 処理の完了まで待つ（決定性の確保）
    auto t2 = std::chrono::steady_clock::now();
    sil_core1_wait_done(rel_before, acq_before);
    auto t3 = std::chrono::steady_clock::now();
    g_last_core1_ns = (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(t3 - t2).count();

    g_tick++;
}

// =========================================================================
// 出力
// =========================================================================
extern "C" void sil_get_duty(float duty[6]) {
    // [FR, FL, MR, ML, RR, RL]
    duty[0] = level_to_duty(sil_pwm_get_level(kSliceFront,  PWM_CHAN_A)); // FR
    duty[1] = level_to_duty(sil_pwm_get_level(kSliceMiddle, PWM_CHAN_A)); // FL
    duty[2] = level_to_duty(sil_pwm_get_level(kSliceRear,   PWM_CHAN_A)); // MR
    duty[3] = level_to_duty(sil_pwm_get_level(kSliceRear,   PWM_CHAN_B)); // ML
    duty[4] = level_to_duty(sil_pwm_get_level(kSliceFront,  PWM_CHAN_B)); // RR
    duty[5] = level_to_duty(sil_pwm_get_level(kSliceMiddle, PWM_CHAN_B)); // RL
}

extern "C" void sil_get_state(float o[24]) {
    o[0]  = Phi - Phi_bias;   o[1]  = Theta - Theta_bias; o[2]  = Psi - Psi_bias;
    o[3]  = Phi_ref;          o[4]  = Theta_ref;          o[5]  = Psi_ref;
    o[6]  = Wp - Pbias;       o[7]  = Wq - Qbias;         o[8]  = Wr - Rbias;
    o[9]  = Pref;             o[10] = Qref;               o[11] = Rref;
    o[12] = P_com;            o[13] = Q_com;              o[14] = R_com;
    o[15] = T_ref;            o[16] = T_stick;            o[17] = altitude;
    o[18] = FR_duty;          o[19] = FL_duty;            o[20] = MR_duty;
    o[21] = ML_duty;          o[22] = RR_duty;            o[23] = RL_duty;
}

extern "C" uint8_t  sil_get_arm_flag(void)     { return Arm_flag; }
extern "C" uint8_t  sil_get_flight_mode(void)  { return Flight_mode; }
extern "C" uint64_t sil_get_time_us(void)      { return time_us_64(); }
extern "C" uint64_t sil_get_last_loop_ns(void) { return g_last_loop_ns; }
extern "C" uint64_t sil_get_last_core1_ns(void){ return g_last_core1_ns; }

extern "C" void sil_get_sched_counts(uint64_t *rel, uint64_t *acq) {
    sil_sched_snapshot(rel, acq);
}

extern "C" void sil_get_debug(float o[20]) {
    o[0]=Ax; o[1]=Ay; o[2]=Az;
    o[3]=Mx; o[4]=My; o[5]=Mz;
    o[6]=Xe(0,0); o[7]=Xe(1,0); o[8]=Xe(2,0); o[9]=Xe(3,0);
    o[10]=MN; o[11]=ME; o[12]=MD;
    o[13]=Phi;       o[14]=Theta;       o[15]=Psi;
    o[16]=Phi_bias;  o[17]=Theta_bias;  o[18]=Psi_bias;
    o[19]=T_ref;
}
