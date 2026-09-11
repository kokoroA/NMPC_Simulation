#include "control.hpp"
#include "rgbled.hpp"

// ============================================================================
// 姿勢 NMPC 切替フラグ
//   定義あり: 姿勢制御を 6 発 Hexa-X NMPC で行う (rate/angle PID を停止)
//   コメントアウト: 従来の 2 段カスケード PID で動作 (即座にロールバック可能)
// ============================================================================
#define USE_NMPC

// モータ/ESC 未接続の机上検証用ログ (姿勢→制御入力、NMPC 計算時間)
//   - リモコン無しでも NMPC を走らせる (Arm_flag==2 で NMPC 常時実行)
//   - 姿勢参照はスティック (Phi_ref / Theta_ref / Psi_ref) に追従
//   - USB シリアルへ 10Hz で CSV 出力 (NMPC_BENCH_SERIAL_LOG)
// NMPC_BENCH_LOG / NMPC_BENCH_SERIAL_LOG は CMakeLists.txt で定義

#ifdef USE_NMPC
#include "nmpc.hpp"
#include "nmpc_model.hpp"
#endif

#ifdef NMPC_WIFI_TELEM
#include "wifi_telemetry.hpp"
#endif

bool first_run = true;
// alt control
// Kalman filter and alt sensor
float i2c_connect = 1;
float stick = 0;
float start_time = time_us_64();
uint64_t count_up = 0;
float Kalman_alt = 0;
float last_Kalman_alt = 0;
float auto_mode_count = 0;
float current_time;
float func_time;
float z_acc;
float base_dis_count = 0;
uint16_t Range;
int8_t status;
uint8_t Temp2;
uint8_t Temp;
uint8_t IntPol;
uint8_t val = 0;
uint8_t range_gbuf[16];
float range_flag = 0;
uint16_t altitude_count = 0;
// float stick;
float auto_mode = 0;
float ideal;
float hove_time = 0.0;
float flying_mode = 0;
float input = 0;
Matrix<float, 3, 3> lotate_mat = MatrixXf::Zero(3, 3);
float f_distance = 0;
float f_distance2 = 0;
float f_distance3 = 0;
float lotated_distance = 0;
Matrix<float, 1, 3> distance_mat = MatrixXf::Zero(1, 3);
Matrix<float, 1, 3> f_distance_mat = MatrixXf::Zero(1, 3);
// OpenMV通信用
char buffer[BUFFER_SIZE];
int buffer_index = 0;
uint8_t release_flag = 0;
uint8_t red_count = 0;
uint16_t loop_count = 0;
float x_1_dash;
float x_2_dash;
float y_1_dash;
float y_2_dash;
float x_diff = 0;
float angle_diff = 0;
int previous_gap_number = 0;
float line_number = 0;
float length = 0;
float x_diff_dash = 0;
float TOL_x_diff_dash = 0;
float TOL_y_diff_dash = 0;
float red_circle = 0;
float TOL_x_diff = 0;
float TOL_y_diff = 0;
float TOL_flag = 0;
float TOL_x_ref = 80;
float TOL_y_ref = 60;
float TOL_x_err = 0;
float TOL_y_err = 0;
float takeoff_counter = 0;
float landing_counter = 0;
float line_trace_flag = 0;
float gap_number = 0;
float T_merker_flag = 0;
float L_merker_flag = 0;
float TOL_x_alpha = 0;
float TOL_y_alpha = 0;
float x_alpha = 0;
float altitude = 0;
int length_count = 0;

// Sensor data
float Ax, Ay, Az, Wp, Wq, Wr, Mx, My, Mz, Mx0, My0, Mz0, Mx_ave, My_ave, Mz_ave;
float Acc_norm = 0.0;
float Line_range = 0.0;
float Line_velocity = 0.0;

// Initial data
float rate_limit = 180.0;

// // Rocking wings
float Rocking_timer = 0.0;
float rocking_wings(float stick);
uint8_t Flight_mode = 0;

// Times
float Elapsed_time = 0.0;
uint32_t S_time = 0, E_time = 0, D_time = 0, S_time2 = 0, E_time2 = 0, D_time2 = 0;

// Counter
uint8_t AngleControlCounter = 0;
uint16_t RateControlCounter = 0;
uint16_t BiasCounter = 0;
uint16_t LedBlinkCounter = 0;
uint16_t LineTraceCounter = 0;
uint16_t Linetrace_counter_for_control = 0;
uint16_t plotnum = 0;
// Control
float FR_duty, FL_duty,MR_duty, ML_duty, RR_duty, RL_duty;
float P_com, Q_com, R_com;
float T_ref;
float T_ref_coe = 0.7;//Benzen_new と同一 (旧: 0.7)
float T_stick;
float Pbias = 0.0, Qbias = 0.0, Rbias = 0.0;
float Phi_bias = 0.0, Theta_bias = 0.0, Psi_bias = 0.0;
float Phi, Theta, Psi = 0.0;
float Phi_ref = 0.0, Theta_ref = 0.0, Psi_ref = 0.0;
float Elevator_center = 0.0, Aileron_center = 0.0, Rudder_center = 0.0;
static uint8_t g_stick_neutral_hold = 0;
static const int STICK_NEUTRAL_MARGIN = 80;
static const uint8_t STICK_NEUTRAL_HOLD_CYCLES = 50;
float Pref = 0.0, Qref = 0.0, Rref = 0.0;
const float Phi_trim = 0.0;
const float Theta_trim = 0.0;
const float Psi_trim = 0.0;
const double pi = 3.14159;
float Line_trace_flag = 0;

// ---- 耐故障(FTC)モード ----
// ★停止させるモータはここ 1 行で変えられる★
//   MOTOR_FR / MOTOR_FL / MOTOR_MR / MOTOR_ML / MOTOR_RR / MOTOR_RL / MOTOR_NONE
int8_t  Ftc_target_motor = MOTOR_FR;
uint8_t Ftc_mode = 0;                                   // 0=健全, 1=FTC
float   Motor_eta[NUM_MOTOR] = {1.f,1.f,1.f,1.f,1.f,1.f};  // 0=故障, 1=健全

// ---- ベンチ加振モード (システム同定用) ----
// ★実験条件はこのブロックの値を書き換えるだけで変えられる★
//   Bench_mode = BENCH_OFF        通常飛行 (既定)
//              = BENCH_YAW_EXCITE ヨー差動加振
//              = BENCH_MOTORS_OFF モータ停止のままログだけ取る (フェーズ0)
// シーケンスはスロットルを Flight_duty 以上に上げた時刻から始まり、
// 下げると頭出しに戻る。スロットルがデッドマンスイッチを兼ねる。
// 手順の全文: docs/20260905_yaw_damping_experiment_protocol.md
uint8_t Bench_mode   = BENCH_OFF;
float   Bench_u0     = 0.60f;   // common duty。6 発同じ値なので合計推力はここで決まる
float   Bench_delta  = 0.05f;   // 差動量。tau_z ≈ 6*C_QT*T'(u0)*delta
float   Bench_sign   = 1.0f;    // 差動の向き。正逆両方を取ると tau_bias が分離できる
float   Bench_t_spin  = 3.0f;   // (1)   u0 のみ。静止確認とロータ立ち上げ [s]
float   Bench_t_step  = 6.0f;   // (2)(3) 差動投入。r を 150-200 deg/s まで育てる [s]
float   Bench_t_decay = 20.0f;  // (4)   差動を 0 に戻して自由減衰 [s]
uint8_t Bench_phase  = 0;
static uint32_t Bench_t0_us = 0;

// ---- フェイルセーフ / 脱出条件 ----
uint8_t Failsafe_flag = 0;
uint8_t Failsafe_cause = FS_NONE;
// 受信途絶とみなす時間 [us]。SBUS は 14ms 周期なので 200ms は 14 フレーム分。
static const uint32_t FS_RX_TIMEOUT_US = 200000;
// 脱出条件。姿勢 [rad] / 角速度 [rad/s] / 加速度 [m/s^2]
// 加速度は LSM9DS1 が ±16g 設定 = 最大 157 m/s^2 なので、旧値 800 は到達不能だった。
static const float FS_ATTITUDE_LIMIT = 90.0f * (float)M_PI / 180.0f;
static const float FS_RATE_LIMIT     = 360.0f * (float)M_PI / 180.0f;
static const float FS_ACC_LIMIT      = 150.0f;  // ≈15g。誤検知で飛行中に止まる方が危険なので高めから始める
// SBUS FRAME_LOST(0x04) は 1 フレーム取りこぼしただけで立つ。単発で空中のモータを永久に切るのは
// 過剰なので、連続してこの本数立ち続けたときだけ発動させる。
// SBUS のフレーム周期は 7ms (高速) / 14ms (標準) なので 16 本 = 約 112-224 ms 相当。
// 本物のリンク断は FS_RX_TIMEOUT (200ms 無受信) が別途捕まえるため、こちらは
// 「フレームは来ているのに中身が失われ続けている」場合の保険という位置づけ。
static const uint8_t FS_FRAMELOST_RUN_LIMIT = 16;

// Extended Kalman filter
Matrix<float, 7, 1> Xp = MatrixXf::Zero(7, 1);
Matrix<float, 7, 1> Xe = MatrixXf::Zero(7, 1);
Matrix<float, 6, 1> Z = MatrixXf::Zero(6, 1);
Matrix<float, 3, 1> Omega_m = MatrixXf::Zero(3, 1);
Matrix<float, 3, 1> Oomega;
Matrix<float, 7, 7> P;
Matrix<float, 6, 6> Q; // = MatrixXf::Identity(3, 3)*0.1;
Matrix<float, 6, 6> R; // = MatrixXf::Identity(6, 6)*0.0001;
Matrix<float, 7, 6> G;
Matrix<float, 3, 1> Beta;

// Log
uint16_t LogdataCounter = 0;
uint8_t Logflag = 0;
volatile uint8_t Logoutputflag = 0;
volatile uint8_t g_arm2_debug_pending = 0;
float Log_time = 0.0;
// ---- 100Hz 機内ログ（リングバッファ）----
// 2026-09-04: 「先頭 14.5 秒を記録して満杯で停止」から「直近 N 秒を常に保持」へ変更。
//   墜落解析では最後の数秒が要るのに、旧実装では離陸直後で満杯になり肝心の場面が残らなかった。
//   列も PID 時代のもの（P_com/altitude/ideal/T_stick/input 等・NMPC では未使用）から
//   NMPC 用（6 発の duty・ソルバ時間・フェイルセーフ）へ差し替えた。
const uint8_t DATANUM = 30;                            // 1 サンプルあたりの列数（2026-09-10: 22 -> 30）
const uint32_t LOGDATANUM = 65280;                     // = 30 * 2176。行数は据え置き（+52 KB）
const uint16_t LOGROWS = (uint16_t)(LOGDATANUM / DATANUM);  // = 2176 行 = 21.8 秒 @100Hz
float Logdata[LOGDATANUM] = {0.0};
static uint16_t LogRow = 0;        // 次に書き込む行
static uint8_t  Log_wrapped = 0;   // 1 = 一周した（バッファ全体が有効）

// State Machine
uint8_t LockMode = 0;
float Disable_duty = 0.1;
float Flight_duty = 0.05; // 0.2///////////////// 0.18
uint8_t OverG_flag = 0;
unsigned short Linetrace_counter = 0;

// PID object and etc.
Filter acc_filter;
PID p_pid;
PID q_pid;
PID r_pid;
PID phi_pid;
PID theta_pid;
PID psi_pid;
PID u_pid;
PID v_pid;
PID y_pid;

Filter Range_filter;
Filter Angle_filter;
Filter Velocity_filter;

#ifdef USE_NMPC
// ---- 姿勢 NMPC (Hexa-X, 100Hz) ----
// NmpcContext は ~200KB のため static (BSS) に 1 度だけ確保する。
static nmpc::NmpcContext g_nmpc_ctx;
static bool     g_nmpc_inited = false;
static bool     g_nmpc_was_running = false;  // angle_control: nmpc_run 立下り検出用
static float    g_nmpc_cost   = 0.0f;   // 直近の評価値 (デバッグ)
// FTC の η が変わったことを core0(rate_control 400Hz) から core1(angle_control 100Hz) へ伝える。
// 1 バイトの立ち上がりだけなのでロック不要 (取りこぼしても次周期で拾う設計にはしない ―
// クリアは core1 側で行い、書き込みは core0 側のみ)。
volatile uint8_t g_motor_eta_dirty = 1;

// ---- 再アーム時の EKF 再初期化と離陸ゲート（2026-09-10）----
// 墜落・転倒すると Arm_flag は 2 -> 3 になるが、**Arm_flag==3 では EKF が 1 度も更新されない**
// （100Hz の angle_control は Arm_flag==1/2 の sem_release でしか回らない）。
// つまり EKF は転倒姿勢のまま凍結し、再アーム（Arm_flag=2）で「転倒姿勢から水平へ収束し始める」。
// 地上では Phi_bias = Phi が毎周期走るので姿勢表示は 0 に見えるが、**収束の速度は消えない**。
// 離陸で Phi_bias が凍結した瞬間、その速度（毎秒 3-8 度）がそのまま「偽の傾き」として現れる。
// 0910 の飛行では、直前が 18° 以上の転倒だった 6 回中 4 回でこれが起きていた。
//   → (1) アーム時に EKF を明示的に再初期化し、(2) 推定が落ち着くまでモータを回さない。
// 詳細: docs/ai/decisions/20260910_v11_post_crash_ekf_and_slow_mode.md
volatile uint8_t g_arm_gate_ok   = 0;   // 1 = 推定器が落ち着いた。0 = モータを回さない
uint32_t g_arm_gate_wait_ms      = 0;   // 直近のアームでゲート通過までに要した時間 [ms]
uint32_t g_arm_reinit_count      = 0;   // アーム時 EKF 再初期化の回数
static uint32_t g_nmpc_iter_us = 0;     // 直近の nmpc_step 所要時間 [us]

// ---- 2026-09-11: 姿勢トリム（積分作用） ----
// 速度比例の外乱（ブローバック）は姿勢 5 次元モデルに無いので、比例項だけでは必ず定常誤差になる。
// 姿勢誤差を積分して U_ref のシフトとして与え、定常の取りこぼしを埋める。
// 単位は「Σ k_j u_j」= 飛行ログで測ってきた指令トルク proxy と同じ。根拠と設計値は
// nmpc_params.hpp の namespace trim のコメント、および
//   docs/ai/decisions/20260911_v8v9_ru_up_loses_stiffness_droop_is_speed_stability.md
float g_trim_roll  = 0.0f;
float g_trim_pitch = 0.0f;
float g_trim_yaw   = 0.0f;

static void att_trim_reset(void)
{
    g_trim_roll = g_trim_pitch = g_trim_yaw = 0.0f;
}

// 100Hz。飛行中かつゲートが開いているときだけ呼ぶこと。
// sat: 直前周期の出力 duty が box に張り付いていたか（張り付いていたら外へ出る向きだけ凍結）。
static void att_trim_update(float e_phi, float e_theta, float e_r, bool sat)
{
    if (!nmpc::trim::enable) { att_trim_reset(); return; }

    const float dt = 0.01f;

    // ---- リーク（減衰）2026-09-11 ----
    // 帯の内外・飽和の有無にかかわらず毎周期引き戻す。これが無いと、積分で消せない外乱に
    // 当たったとき上限に張り付いたまま戻らず、「指令から一定量ずれた目標を追い続ける」状態になる
    // （0911_v11 で trim_roll が +6 deg に張り付いたまま飛行が終わった）。
    // 直流ゲインが ki*leak_tau に有限化されるので、消せない分は素直に定常誤差として残る。
    {
        const float aa = 1.0f - dt / nmpc::trim::att_leak_tau;
        const float ay = 1.0f - dt / nmpc::trim::yaw_leak_tau;
        g_trim_roll  *= aa;
        g_trim_pitch *= aa;
        g_trim_yaw   *= ay;
    }
    // 大きく姿勢を崩している間は積まない（巻き上がり防止）
    const bool ok_phi   = (fabsf(e_phi)   < nmpc::trim::att_err_max);
    const bool ok_theta = (fabsf(e_theta) < nmpc::trim::att_err_max);
    const bool ok_r     = (fabsf(e_r)     < nmpc::trim::yaw_err_max);

    // 飽和中は「さらに外へ出る向き」だけ止める（戻る向きは常に許す）= 標準的なアンチワインドアップ
    auto integrate = [&](float &trim, float err, float ki, float lim, bool ok) {
        if (!ok) return;
        const float d = ki * err * dt;
        if (sat && ((d > 0.0f && trim > 0.0f) || (d < 0.0f && trim < 0.0f))) return;
        float t = trim + d;
        if (t >  lim) t =  lim;
        if (t < -lim) t = -lim;
        trim = t;
    };
    integrate(g_trim_roll,  e_phi,   nmpc::trim::att_ki, nmpc::trim::att_max, ok_phi);
    integrate(g_trim_pitch, e_theta, nmpc::trim::att_ki, nmpc::trim::att_max, ok_theta);
    integrate(g_trim_yaw,   e_r,     nmpc::trim::yaw_ki, nmpc::trim::yaw_max, ok_r);
}

// トリムを状態参照 X_ref のオフセットとして適用する。
// ⚠ U_ref（入力の参照）をずらす実装は効かなかった（Q が支配的だとソルバが潰す）。
//   経緯と実測値は nmpc_params.hpp の namespace trim のコメント。
static void att_trim_apply_ref(float &phi_ref, float &theta_ref, float &r_ref)
{
    if (!nmpc::trim::enable) return;
    phi_ref   += g_trim_roll;
    theta_ref += g_trim_pitch;
    r_ref     += g_trim_yaw;
}

// 非武装/着地時に warm-start を破棄して次回フル再構築させる
void nmpc_reset_request(void)
{
    nmpc::nmpc_mark_external_rebuild(g_nmpc_ctx);
}

#if NMPC_BENCH_LOG
// core1(angle_control) が更新、core0(main) が読むスナップショット
struct NmpcBenchSnapshot {
    volatile uint32_t seq;
    volatile uint8_t  arm_flag;
    volatile uint8_t  nmpc_active;   // 1 = 今周期 NMPC を実行した
    volatile uint8_t  nmpc_reset;   // 直近 step でフル再構築したか
    volatile uint8_t  reset_cause;  // NMPC_REBUILD_* (直近 step)
    volatile uint8_t  step_path;    // NMPC_PATH_* (直近 step)
    volatile uint8_t  tr1_applied;  // 直近 TR1/SR2 経路で TR1 が適用されたか
    volatile int      cycles_since_reset;
    volatile uint32_t cnt_tr1_path;
    volatile uint32_t cnt_tr1_applied;
    volatile uint32_t cnt_tr1_skipped;
    volatile uint32_t cnt_reset_period;
    volatile uint32_t cnt_reset_external;
    volatile uint32_t cnt_reset_sr2_bad_c;
    volatile uint32_t cnt_reset_sr2_unhealthy;
    volatile uint32_t cnt_reset_after_sr2;
    volatile float phi;
    volatile float theta;
    volatile float p;
    volatile float q;
    volatile float r;
    volatile float phi_ref;
    volatile float theta_ref;
    volatile float psi_ref;
    volatile float t_ref;
    volatile float u_base;          // u_hover baseline
    volatile float schedule_alpha;  // u_hover scheduling 係数 [0,1]
    volatile float Q_phi_eff;       // 実効 Q_phi (scheduled)
    volatile float u_fr, u_fl, u_mr, u_ml, u_rr, u_rl;
    volatile float iter_us;         // nmpc_step 所要時間 [us]
    volatile float loop_us;         // angle_control 1 周期 [us]
    volatile float cost;
    volatile float iter_us_max;     // 起動後の最大 iter_us
    volatile uint8_t ftc;           // 耐故障モード 0=健全 / 1=FTC
    volatile uint8_t failsafe;      // FS_* の原因コード (0=正常)
    volatile uint8_t rx_flags;      // SBUS フラグバイト生値 (0x04=frame lost, 0x08=failsafe)
    volatile uint8_t rx_fl_run;     // FRAME_LOST が連続しているフレーム数
    volatile uint32_t rx_fl_cnt;    // FRAME_LOST の累積回数
    // ---- 2026-09-11 追加: 状態機械の観測性 ----
    // 0911_v7 の「LED の色が変わらない / モータが回らない」を切り分けるのに必要だったが、
    // どれもログに出ていなかった。詳細:
    //   docs/ai/decisions/20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md
    volatile uint8_t  flight_mode;  // Flight_mode (0=NORMAL 1=MODE_HOVERING 2=MODE_FTC)
    volatile uint8_t  lock_mode;    // LockMode (0..3) アーム/ディスアーム手順の段階
    volatile uint8_t  gate;         // g_arm_gate_ok (0=モータ出力禁止中)
    volatile float    trim_roll;    // 姿勢トリム（積分作用）Σ(-kYm·u) 単位
    volatile float    trim_pitch;   // Σ(+kXm·u) 単位
    volatile float    trim_yaw;     // Σ(kS·u) 単位
    volatile uint16_t ch_ftc;       // Chdata[FTC] 生値（しきい値 <200 / >500 の判定元）
};

static NmpcBenchSnapshot g_nmpc_bench = {};
#if !defined(NMPC_BENCH_SERIAL_LOG) || NMPC_BENCH_SERIAL_LOG
static bool g_nmpc_bench_header_printed = false;
#endif

static void nmpc_bench_update(const NmpcBenchSnapshot &snap)
{
    const float prev_max = g_nmpc_bench.iter_us_max;
    g_nmpc_bench = snap;
    g_nmpc_bench.iter_us_max = (snap.iter_us > prev_max) ? snap.iter_us : prev_max;
}

#ifdef NMPC_WIFI_TELEM
static bool nmpc_bench_fill_tel(NmpcTelPkt *out)
{
    static uint32_t last_seq = 0;
    if (g_nmpc_bench.seq == last_seq)
        return false;
    last_seq = g_nmpc_bench.seq;

    const float ub = g_nmpc_bench.u_base;
    out->hdr.magic = NMPC_UDP_MAGIC;
    out->hdr.type = NMPC_PKT_TEL;
    out->hdr.ver = NMPC_UDP_VER;
    out->hdr.seq = last_seq;
    out->arm_flag = g_nmpc_bench.arm_flag;
    out->nmpc_active = g_nmpc_bench.nmpc_active;
    out->nmpc_reset = g_nmpc_bench.nmpc_reset;
    out->reset_cause = g_nmpc_bench.reset_cause;
    out->step_path = g_nmpc_bench.step_path;
    out->tr1_applied = g_nmpc_bench.tr1_applied;
    out->ftc = g_nmpc_bench.ftc;
    out->failsafe = g_nmpc_bench.failsafe;
    out->rx_flags = g_nmpc_bench.rx_flags;
    out->rx_framelost_run = g_nmpc_bench.rx_fl_run;
    out->_rsv16 = 0;
    out->rx_framelost_cnt = g_nmpc_bench.rx_fl_cnt;
    out->cycles_since_reset = g_nmpc_bench.cycles_since_reset;
    out->cnt_tr1_path = g_nmpc_bench.cnt_tr1_path;
    out->cnt_tr1_applied = g_nmpc_bench.cnt_tr1_applied;
    out->cnt_tr1_skipped = g_nmpc_bench.cnt_tr1_skipped;
    out->cnt_reset_period = g_nmpc_bench.cnt_reset_period;
    out->cnt_reset_external = g_nmpc_bench.cnt_reset_external;
    out->cnt_reset_sr2_bad_c = g_nmpc_bench.cnt_reset_sr2_bad_c;
    out->cnt_reset_sr2_unhealthy = g_nmpc_bench.cnt_reset_sr2_unhealthy;
    out->cnt_reset_after_sr2 = g_nmpc_bench.cnt_reset_after_sr2;
    out->phi = g_nmpc_bench.phi;
    out->theta = g_nmpc_bench.theta;
    out->p = g_nmpc_bench.p;
    out->q = g_nmpc_bench.q;
    out->r = g_nmpc_bench.r;
    out->phi_ref = g_nmpc_bench.phi_ref;
    out->theta_ref = g_nmpc_bench.theta_ref;
    out->psi_ref = g_nmpc_bench.psi_ref;
    out->T_ref = g_nmpc_bench.t_ref;
    out->u_base = ub;
    out->schedule_alpha = g_nmpc_bench.schedule_alpha;
    out->iter_us = g_nmpc_bench.iter_us;
    out->loop_us = g_nmpc_bench.loop_us;
    out->cost = g_nmpc_bench.cost;
    out->iter_us_max = g_nmpc_bench.iter_us_max;
    out->u_fr = g_nmpc_bench.u_fr;
    out->u_fl = g_nmpc_bench.u_fl;
    out->u_mr = g_nmpc_bench.u_mr;
    out->u_ml = g_nmpc_bench.u_ml;
    out->u_rr = g_nmpc_bench.u_rr;
    out->u_rl = g_nmpc_bench.u_rl;
    out->d_fr = g_nmpc_bench.u_fr - ub;
    out->d_fl = g_nmpc_bench.u_fl - ub;
    out->d_mr = g_nmpc_bench.u_mr - ub;
    out->d_ml = g_nmpc_bench.u_ml - ub;
    out->d_rr = g_nmpc_bench.u_rr - ub;
    out->d_rl = g_nmpc_bench.u_rl - ub;

    nmpc::NmpcCostParams cp = nmpc::nmpc_cost_get_active();
    out->Q_phi = cp.Q_phi;
    out->Q_theta = cp.Q_theta;
    out->Q_p = cp.Q_p;
    out->Q_q = cp.Q_q;
    out->Q_r = cp.Q_r;
    out->R_u = cp.R_u;
    out->terminal_scale = cp.terminal_scale;
    return true;
}
#endif

void debug_print_nmpc_bench(void)
{
#if !defined(NMPC_BENCH_SERIAL_LOG) || NMPC_BENCH_SERIAL_LOG
    if (!g_nmpc_bench_header_printed) {
        g_nmpc_bench_header_printed = true;
        printf("#nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)\r\n");
        printf("#reset_cause: 0=none 1=init 2=external 3=period 4=sr2_bad_c 5=sr2_unhealthy\r\n");
        printf("#failsafe: 0=none 1=rx_lost 2=rx_timeout 3=attitude 4=rate 5=overg\r\n");
        printf("#path: 0=tr1/sr2 1=reset@start 2=reset@after_sr2\r\n");
        printf("#flight_mode: 0=NORMAL 1=MODE_HOVERING(未使用) 2=MODE_FTC\r\n");
        printf("#lock_mode: 0=待機 1=アーム操作中 2=飛行可 3=ディスアーム操作中\r\n");
        printf("#trim: 姿勢トリム(積分作用)。単位は指令トルク proxy と同じ Σk·u。0 = 無効/未蓄積\r\n");
        printf("#gate: 1=離陸ゲート開(モータ出力可) 0=閉(アーム後 2.5s または操作中)\r\n");
        printf("#cols: seq,arm,nmpc,phi,theta,p,q,r,phi_ref,theta_ref,psi_ref,T_ref,u_base,"
               "schedule_alpha,Q_phi_eff,"
               "iter_us,loop_us,cost,reset,iter_max,"
               "reset_cause,path,tr1,cycles,"
               "cnt_tr1,cnt_tr1_ok,cnt_tr1_skip,"
               "cnt_rst_period,cnt_rst_ext,cnt_sr2_c,cnt_sr2_llt,cnt_rst_after_sr2,"
               "u_fr,u_fl,u_mr,u_ml,u_rr,u_rl,"
               "d_fr,d_fl,d_mr,d_ml,d_rr,d_rl,"
               "ftc,failsafe,rx_flags,rx_fl_run,rx_fl_cnt,"
               "flight_mode,lock_mode,gate,ch_ftc,"
               "trim_roll,trim_pitch,trim_yaw\r\n");
    }

    const float ub = g_nmpc_bench.u_base;
    printf("%lu,%u,%u,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.4f,%.4f,"
           "%.0f,%.0f,%.2f,%u,%.0f,"
           "%u,%u,%u,%d,"
           "%lu,%lu,%lu,"
           "%lu,%lu,%lu,%lu,%lu,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,"
           "%u,%u,%u,%u,%lu,"
           "%u,%u,%u,%u,"
           "%.5f,%.5f,%.5f\r\n",
           (unsigned long)g_nmpc_bench.seq,
           (unsigned)g_nmpc_bench.arm_flag,
           (unsigned)g_nmpc_bench.nmpc_active,
           g_nmpc_bench.phi, g_nmpc_bench.theta,
           g_nmpc_bench.p, g_nmpc_bench.q, g_nmpc_bench.r,
           g_nmpc_bench.phi_ref, g_nmpc_bench.theta_ref,
           g_nmpc_bench.psi_ref, g_nmpc_bench.t_ref, ub,
           g_nmpc_bench.schedule_alpha, g_nmpc_bench.Q_phi_eff,
           g_nmpc_bench.iter_us, g_nmpc_bench.loop_us,
           g_nmpc_bench.cost,
           (unsigned)g_nmpc_bench.nmpc_reset,
           g_nmpc_bench.iter_us_max,
           (unsigned)g_nmpc_bench.reset_cause,
           (unsigned)g_nmpc_bench.step_path,
           (unsigned)g_nmpc_bench.tr1_applied,
           g_nmpc_bench.cycles_since_reset,
           (unsigned long)g_nmpc_bench.cnt_tr1_path,
           (unsigned long)g_nmpc_bench.cnt_tr1_applied,
           (unsigned long)g_nmpc_bench.cnt_tr1_skipped,
           (unsigned long)g_nmpc_bench.cnt_reset_period,
           (unsigned long)g_nmpc_bench.cnt_reset_external,
           (unsigned long)g_nmpc_bench.cnt_reset_sr2_bad_c,
           (unsigned long)g_nmpc_bench.cnt_reset_sr2_unhealthy,
           (unsigned long)g_nmpc_bench.cnt_reset_after_sr2,
           g_nmpc_bench.u_fr, g_nmpc_bench.u_fl,
           g_nmpc_bench.u_mr, g_nmpc_bench.u_ml,
           g_nmpc_bench.u_rr, g_nmpc_bench.u_rl,
           g_nmpc_bench.u_fr - ub, g_nmpc_bench.u_fl - ub,
           g_nmpc_bench.u_mr - ub, g_nmpc_bench.u_ml - ub,
           g_nmpc_bench.u_rr - ub, g_nmpc_bench.u_rl - ub,
           (unsigned)g_nmpc_bench.ftc,
           (unsigned)g_nmpc_bench.failsafe,
           (unsigned)g_nmpc_bench.rx_flags,
           (unsigned)g_nmpc_bench.rx_fl_run,
           (unsigned long)g_nmpc_bench.rx_fl_cnt,
           (unsigned)g_nmpc_bench.flight_mode,
           (unsigned)g_nmpc_bench.lock_mode,
           (unsigned)g_nmpc_bench.gate,
           (unsigned)g_nmpc_bench.ch_ftc,
           g_nmpc_bench.trim_roll,
           g_nmpc_bench.trim_pitch,
           g_nmpc_bench.trim_yaw);
#endif  // NMPC_BENCH_SERIAL_LOG

#ifdef NMPC_WIFI_TELEM
    if (wifi_telemetry_ready()) {
        NmpcTelPkt tel;
        if (nmpc_bench_fill_tel(&tel))
            wifi_telemetry_send_bench(tel);
    }
#endif
}
#else
void debug_print_nmpc_bench(void) {}
#endif  // NMPC_BENCH_LOG
#else
void debug_print_nmpc_bench(void) {}
#endif  // USE_NMPC

void debug_poll_pending_messages(void)
{
  if (g_arm2_debug_pending)
  {
    g_arm2_debug_pending = 0;
    printf("#debug: skip lock_com, Arm_flag=2\r\n");
  }
}

void loop_400Hz(void);
void rate_control(void);
void sensor_read(void);
void angle_control(void);
void output_data(void);
void output_sensor_raw_data(void);
void kalman_filter(void);
void logging(void);
void motor_stop(void);
static void failsafe_check(void);
static void ekf_reinit_on_arm(void);
uint8_t lock_com(void);
uint8_t logdata_out_com(void);
static void latch_stick_centers(void);
static uint8_t sticks_neutral(void);
static float stick_ch_center(uint16_t ch_min, uint16_t ch_max, float stored_center);
static void update_attitude_refs_from_sticks(void);
void printPQR(void);
// 追加
void servo_control(void);
void led_control(void);
void linetrace(void);
// alt control
void Merker_Hovering(void);
void Auto_fly(void);
void Auto_takeoff(void);
void Auto_landing(void);
void Hovering(void);
float lotate_altitude(float l_distance);
void lotate_altitude_init(float Theta, float Psi, float Phi);
void processReceiveData();
void uart_read(uint8_t *buffer, size_t len);
void receiveData(char c);
void takeoff_merker(void);
void landing_merker(void);
void send_data_via_uart(void);
void receiveData(char c);

#define AVERAGE 2000
#define KALMANWAIT 6000

// // LED Control
void led_control(void)
{
  static uint16_t cnt = 0;
  if (Arm_flag == 0 || Arm_flag == 1)
    rgbled_wait();

  // ベンチ加振モード: 全周ピンクの速い点滅 (10Hz)。
  // 通常飛行と取り違えると危険なので、どのモードよりも先に判定する。
  else if (Arm_flag == 2 && Bench_mode != BENCH_OFF)
  {
    if (cnt == 0)
      rgbled_pink();
    if (cnt == 20)
      rgbled_off();
    cnt++;
    if (cnt == 40)
      cnt = 0;
  }

  // 離陸ゲート中: 全周赤の速い点滅 (5Hz)。**この間モータは回らない。**
  // 推定器が転倒姿勢から収束し切るのを待っている状態。通常は 2-4 秒で消える。
  // 消えない場合は推定器かセンサに異常があるので、飛ばさずに電源を入れ直すこと。
  else if (Arm_flag == 2 && !g_arm_gate_ok)
  {
    if (cnt == 0)
      rgbled_red();
    if (cnt == 40)
      rgbled_off();
    cnt++;
    if (cnt == 80)
      cnt = 0;
  }

  // 耐故障(FTC)モード: 通常表示 ⇔ 全周青 の交互 (4Hz)。
  // Flight_mode より先に判定して、どのモードでも FTC 中は必ずこの表示になるようにする。
  else if (Arm_flag == 2 && Ftc_mode)
  {
    if (cnt == 0)
      rgbled_normal();
    if (cnt == 50)
      rgbled_blue();
    cnt++;
    if (cnt == 100)
      cnt = 0;
  }
  else if (Arm_flag == 2 && Flight_mode == NORMAL)
    rgbled_normal();
  else if (Arm_flag == 2 && Flight_mode == MODE_HOVERING)
  {
    if (cnt == 0)
      rgbled_normal();
    if (cnt == 50)
      rgbled_pink();
    cnt++;
    if (cnt == 100)
      cnt = 0;
  }
  // REDCIRCLE モードは削除 (ch5 は FTC スイッチへ転用。red_circle は外部 UART 由来で未接続)
  // else if (Arm_flag == 2 && Red_flag == 0 && Logflag == 1) rgbled_orange();

  else if (Arm_flag == 3)
  {
    if (cnt == 0)
      rgbled_green();
    if (cnt == 50)
      rgbled_off();
    cnt++;
    if (cnt == 100)
      cnt = 0;
  }
}

// Main loop
// This function is called from PWM Intrupt on 400Hz.
void loop_400Hz(void)
{
  static uint8_t led = 1;
  S_time = time_us_32();

  // 割り込みフラグリセット
  pwm_clear_irq(7);

  // Servo Control
  servo_control();

  // LED Control
  led_control();

  // ---- フェイルセーフの解除 (必ず Arm_flag の分岐より前で行う) ----
  // failsafe_check() は Arm_flag==2 の枝の中からしか呼ばれないため、その内部にある
  // 「Arm_flag != 2 なら解除」には制御が到達しない。結果として一度成立すると
  // 再アームしても解除できず、モータが二度と回らなかった。
  if (Arm_flag != 2)
  {
    Failsafe_flag = 0;
    Failsafe_cause = FS_NONE;
    Rx_framelost_run = 0;   // リンクが死んだまま再アームしても即発動しないようにする
  }

  if (Arm_flag == 0)
  {
    // motor_stop();
    Elevator_center = 0.0;
    Aileron_center = 0.0;
    Rudder_center = 0.0;
    g_stick_neutral_hold = 0;
    Pbias = 0.0;
    Qbias = 0.0;
    Rbias = 0.0;
    Phi_bias = 0.0;
    Theta_bias = 0.0;
    Psi_bias = 0.0;
    return;
  }
  else if (Arm_flag == 1)
  {
    motor_stop();
    // Gyro Bias Estimate
    if (BiasCounter < AVERAGE)
    {
      // Sensor Read
      sensor_read();
      // Aileron_center  += Chdata[3];
      // Elevator_center += Chdata[1];
      // Rudder_center   += Chdata[0];
      Pbias += Wp;
      Qbias += Wq;
      Rbias += Wr;
      Mx_ave += Mx;
      My_ave += My;
      Mz_ave += Mz;
      BiasCounter++;
      return;
    }
    else if (BiasCounter < KALMANWAIT)
    {
      // Sensor Read
      sensor_read();
      if (BiasCounter == AVERAGE)
      {
        // Elevator_center = Elevator_center/AVERAGE;
        // Aileron_center  = Aileron_center/AVERAGE;
        // Rudder_center   = Rudder_center/AVERAGE;
        Pbias = Pbias / AVERAGE;
        Qbias = Qbias / AVERAGE;
        Rbias = Rbias / AVERAGE;
        Mx_ave = Mx_ave / AVERAGE;
        My_ave = My_ave / AVERAGE;
        Mz_ave = Mz_ave / AVERAGE;

        Xe(4, 0) = Pbias;
        Xe(5, 0) = Qbias;
        Xe(6, 0) = Rbias;
        Xp(4, 0) = Pbias;
        Xp(5, 0) = Qbias;
        Xp(6, 0) = Rbias;
        MN = Mx_ave;
        ME = My_ave;
        MD = Mz_ave;
      }

      AngleControlCounter++;
      if (AngleControlCounter == 4)
      {
        AngleControlCounter = 0;
        sem_release(&sem);
      }
      Phi_bias += Phi;
      Theta_bias += Theta;
      Psi_bias += Psi;
      BiasCounter++;
      return;
    }
    else
    {
      Phi_bias = Phi_bias / KALMANWAIT;
      Theta_bias = Theta_bias / KALMANWAIT;
      Psi_bias = Psi_bias / KALMANWAIT;
#if DEBUG_SKIP_LOCK_COM
      latch_stick_centers();
      LockMode = 2;
      Arm_flag = 2;
      ekf_reinit_on_arm();
      g_arm2_debug_pending = 1;  // printf は 400Hz IRQ 内不可 → main で出力
#else
      Arm_flag = 3;
#endif
      return;
    }
  }
  else if (Arm_flag == 2)
  {
    if (LockMode == 2)
    {
      if (lock_com() == 1)
      {
        LockMode = 3; // Disenable Flight
        g_arm_gate_ok = 0;   // 次のアームでも必ずゲートを通す
        led = 0;
        board_led_request(led);
        return;
      }
      // Goto Flight
    }
    else if (LockMode == 3)
    {
      if (lock_com() == 0)
      {
        LockMode = 0;
        Arm_flag = 3;
      }
      return;
    }
    // LED Blink
    board_led_request(led);
    if (Logflag == 1 && LedBlinkCounter < 100)
    {
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter = 0;
      led = !led;
    }

    // フェイルセーフ / 脱出条件 (400Hz)
    failsafe_check();

    // Rate Control (400Hz)
    rate_control();
    if (AngleControlCounter == 4)
    {
      AngleControlCounter = 0;
      // Angle Control (100Hz)
      sem_release(&sem);
    }
    // if(LineTraceCounter == 10)
    //{
    //   LineTraceCounter = 0;
    //   //linetrace (40Hz)
    //   if (Line_trace_flag == 1){
    //     linetrace();
    //   }
    // }
    AngleControlCounter++;
    LineTraceCounter++;
  }
  else if (Arm_flag == 3)
  {
    motor_stop();
    OverG_flag = 0;
    if (LedBlinkCounter < 10)
    {
      board_led_request(1);
      LedBlinkCounter++;
    }
    else if (LedBlinkCounter < 100)
    {
      board_led_request(0);
      LedBlinkCounter++;
    }
    else
      LedBlinkCounter = 0;

    if (LockMode == 0)
    {
      g_stick_neutral_hold = 0;
      if (lock_com() == 1)
      {
        LockMode = 1;
        return;
      }
      // Wait  output log
    }
    else if (LockMode == 1)
    {
      if (lock_com() == 0)
      {
        if (sticks_neutral())
        {
          if (g_stick_neutral_hold < STICK_NEUTRAL_HOLD_CYCLES)
            g_stick_neutral_hold++;
          if (g_stick_neutral_hold >= STICK_NEUTRAL_HOLD_CYCLES)
          {
            latch_stick_centers();
            LockMode = 2; // Enable Flight
            Arm_flag = 2;
            g_stick_neutral_hold = 0;
            ekf_reinit_on_arm();   // 前の飛行の姿勢を引きずらせない（離陸ゲートが開くまで出力しない）
          }
        }
        else
          g_stick_neutral_hold = 0;
      }
      else
        g_stick_neutral_hold = 0;
      return;
    }

    if (logdata_out_com() == 1)
    {
      Arm_flag = 4;
      return;
    }
  }
  else if (Arm_flag == 4)
  {
    motor_stop();
    Logoutputflag = 1;
    // LED Blink
    rgbled_switch(led);
    board_led_request(led);
    if (LedBlinkCounter < 400)
    {
      LedBlinkCounter++;
    }
    else
    {
      LedBlinkCounter = 0;
      led = !led;
    }
  }
  E_time = time_us_32();
  D_time = E_time - S_time;
}

// void send_data_via_uart(const char* data) {
//     while (*data != '\0') {
//         uart_putc(UART_ID2, *data);
//         data++;
//     }
// }

void control_init(void)
{
  acc_filter.set_parameter(0.005, 0.0025);
  Range_filter.set_parameter(0.08, 0.025);
  Angle_filter.set_parameter(0.08, 0.025);
  Velocity_filter.set_parameter(0.08, 0.025);

  // Rate control
  p_pid.set_parameter(0.025, 100000.0, 0, 0.125, 0.0025); //ikaring(2.2, 5, 0.01) itocopter(2.5, 100, 0.009)
  q_pid.set_parameter(0.03, 100000.0, 0, 0.125, 0.0025); //ikaring(1.5, 1, 0.01) itocopter(2.5, 100, 0.009)
  r_pid.set_parameter(0.1, 100000.0, 0, 0.125, 0.0025);  //ikaring(3.1, 1, 0.01) itocopter(3.5, 10, 0.009)
  // Angle control
  phi_pid.set_parameter(1, 100000.0, 0, 0.125, 0.01);   // 6.0  8.0,20,0.007
  theta_pid.set_parameter(1.6, 10000.0, 0.0007, 0.125, 0.01); // 6.0  8.0,20,0.007       //theta p:1.6
  psi_pid.set_parameter(0, 10000, 0, 0, 0.01);      //0 1000 0.01

  // Linetrace
  // velocity control
  v_pid.set_parameter(0.00025, 100000, 0.01, 0.125, 0.03); // 0.0002,100000,0.01

#ifdef USE_NMPC
  // 姿勢 NMPC 初期化 + モデルのサニティチェック
  nmpc::nmpc_cost_init();
  nmpc::nmpc_init(g_nmpc_ctx);
  nmpc::nmpc_warmup(g_nmpc_ctx);
  g_nmpc_inited = true;
  bool ok = nmpc::model_sanity();
  printf("#NMPC init: NU=%d N=%d NU_TOTAL=%d reset_period=%d model_sanity=%s\r\n",
         nmpc::mpc::NU, nmpc::mpc::N, nmpc::mpc::NU_TOTAL,
         nmpc::rti::reset_period,
         ok ? "PASS" : "FAIL");
#endif
}

static void latch_stick_centers(void)
{
  Aileron_center = (float)Chdata[3];
  Elevator_center = (float)Chdata[1];
  Rudder_center = (float)Chdata[0];
}

static int stick_delta_from_mid(uint16_t ch, int mid)
{
  int d = (int)ch - mid;
  return (d < 0) ? -d : d;
}

static uint8_t sticks_neutral(void)
{
  const int mid1 = (CH1MAX + CH1MIN) / 2;
  const int mid2 = (CH2MAX + CH2MIN) / 2;
  const int mid4 = (CH4MAX + CH4MIN) / 2;
  if (Chdata[2] >= CH3MIN + 400)
    return 0;
  if (stick_delta_from_mid(Chdata[0], mid1) > STICK_NEUTRAL_MARGIN)
    return 0;
  if (stick_delta_from_mid(Chdata[1], mid2) > STICK_NEUTRAL_MARGIN)
    return 0;
  if (stick_delta_from_mid(Chdata[3], mid4) > STICK_NEUTRAL_MARGIN)
    return 0;
  return 1;
}

static float stick_ch_center(uint16_t ch_min, uint16_t ch_max, float stored_center)
{
  if (stored_center >= (float)ch_min && stored_center <= (float)ch_max)
    return stored_center;
  return (float)(ch_min + ch_max) * 0.5f;
}

static void update_attitude_refs_from_sticks(void)
{
  const float ail_c = stick_ch_center(CH4MIN, CH4MAX, Aileron_center);
  const float ele_c = stick_ch_center(CH2MIN, CH2MAX, Elevator_center);
  const float rud_c = stick_ch_center(CH1MIN, CH1MAX, Rudder_center);
  if (Flight_mode != ROCKING)
  {
    Phi_ref = Phi_trim + 0.3f * (float)M_PI * (float)(Chdata[3] - ail_c) * 2.0f
              / (float)(CH4MAX - CH4MIN);
  }
  Theta_ref = Theta_trim + 0.3f * (float)M_PI * (float)(Chdata[1] - ele_c) * 2.0f
              / (float)(CH2MAX - CH2MIN);
  // 2026-09-04: 0.8f → 0.3f (±144 → ±54 deg/s)。ロール/ピッチと同じスケールに揃える。
  // 0904_v1 でヨーが Δu の 84.7% を消費し、上限飽和 39% を起こしてロール/ピッチを飢えさせていた。
  // 詳細: docs/20260904_nmpc_yaw_starves_roll_pitch.md
  Psi_ref = Psi_trim + 0.3f * (float)M_PI * (float)(Chdata[0] - rud_c) * 2.0f
            / (float)(CH1MAX - CH1MIN);
}

uint8_t lock_com(void)
{
  static uint8_t chatta = 0, state = 0;
  if (Chdata[2] < CH3MIN + 400 && Chdata[0] > CH1MAX - 400 && Chdata[3] < CH4MIN + 400 && Chdata[1] > CH2MAX - 400) //80
  {
    chatta++;
    if (chatta > 50)
    {
      chatta = 50;
      state = 1;
    }
  }
  else
  {
    chatta = 0;
    state = 0;
  }

  return state;
}

uint8_t logdata_out_com(void)
{
  static uint8_t chatta = 0, state = 0;
  if (Chdata[4] < (CH5MAX + CH5MIN) * 0.5 && Chdata[2] < CH3MIN + 80 && Chdata[0] < CH1MIN + 80 && Chdata[3] > CH4MAX - 80 && Chdata[1] > CH2MAX - 80)
  {
    chatta++;
    if (chatta > 50)
    {
      chatta = 50;
      state = 1;
    }
  }
  else
  {
    chatta = 0;
    state = 0;
  }

  return state;
}

// ---- 安全弁 3: 姿勢推定 (EKF) の破綻検出と再初期化 ----
// 墜落の衝撃などで EKF が発散すると x0 が異常値になり (0909_v1 実測: phi = -336 deg)、
// NMPC が NaN を出す。duty が NaN になると 400Hz の NaN ガードが minimum_duty=0.1 に
// 置き換えるため、スロットルに関係なく全 6 発が 0.1 で回り続ける固着になる。
// 再アームでも EKF が壊れたままなので直らず、電源を抜くまで復帰しなかった。
uint32_t Ekf_reset_count = 0;

// ---- 遅れ補償用の出力 duty 履歴（2026-09-10）----
#define G_DUTY_HIST_N 8
static float    g_duty_hist[G_DUTY_HIST_N][NUM_MOTOR];
static int      g_duty_hist_idx = 0;            // 次に書く位置
uint32_t        g_delay_pred_nonfinite = 0;     // 予測が非有限になり x0 をそのまま使った回数
static void duty_hist_push(void)
{
  g_duty_hist[g_duty_hist_idx][0] = FR_duty;
  g_duty_hist[g_duty_hist_idx][1] = FL_duty;
  g_duty_hist[g_duty_hist_idx][2] = MR_duty;
  g_duty_hist[g_duty_hist_idx][3] = ML_duty;
  g_duty_hist[g_duty_hist_idx][4] = RR_duty;
  g_duty_hist[g_duty_hist_idx][5] = RL_duty;
  g_duty_hist_idx = (g_duty_hist_idx + 1) % G_DUTY_HIST_N;
}

// ---- EKF バイアス発散の検知（2026-09-10）----
// 0910_v2 / 0910_v5 で「ジャイロはゼロなのに姿勢角だけが毎秒 7-8 度ランプする」事象が 3 回出た。
// EKF の状態 Xe(4..6)（ジャイロバイアス、beta=0 のランダムウォーク）が流れると、姿勢出力だけが
// 積分誤差でランプし、NMPC が見る p,q は静止平均基準 (Wp - Pbias) なのでゼロのまま、という形になる。
// 検知: EKF 姿勢の時間微分と、ジャイロから作ったオイラー角速度の差を LPF し、上限を超えたら
// **バイアス状態だけ**を起動時静止平均へ戻す。四元数は触らないので姿勢に段差は出ず、飛行中でも使える。
// （全リセット ekf_reinit() は非有限・桁外れ用の最後の砦として別に残す）
uint32_t Ekf_bias_reset_count = 0;
float    Ekf_drift_phi   = 0.0f;   // [rad/s] LPF 後の残差 (d(Phi)/dt - ジャイロ由来 phi_dot)
float    Ekf_drift_theta = 0.0f;   // [rad/s]
// 閾値の根拠（全ログでオフライン再現、docs/ai/decisions/20260910_ekf_guard_solver_log_and_gain_match.md）:
//   3 deg/s/0.1 s : 飛べていた 0904_v2/v4/v5 の飛行中に 2-5 回誤発火（LPF 残差 3-4 deg/s は正常範囲）
//   5 deg/s/0.2 s : 飛行中の誤発火 0（0904_v5 の着陸バウンド時 1 回のみ）。
//                   偽の傾き 2 件は離陸の約 1 秒前（18.4 s / 10.7 s）に検知できる。
static const float EKF_DRIFT_LIMIT   = 5.0f * (float)M_PI / 180.0f;  // 5 deg/s
static const float EKF_DRIFT_LPF_A   = 0.0323f;                        // τ≈0.3 s @100Hz
static const uint8_t EKF_DRIFT_RUN_N = 20;                             // 0.2 s 連続で超えたら発火

// ---- 離陸ゲートの判定値 ----
// 正常な地上静止では残差は ±1 deg/s 以内に収まる（0910 の全ログで確認）。
// LPF は τ=0.3 s なので、再初期化から 1.5 s（5τ）は判定を始めない。
// そうしないと「LPF の初期値 0」を「落ち着いている」と誤判定して即座にゲートが開く。
static const float    EKF_GATE_LIMIT   = 1.0f * (float)M_PI / 180.0f;  // 1 deg/s
static const uint16_t EKF_GATE_HOLD_N  = 100;                          // 1.0 s 連続で条件成立
static const uint32_t EKF_GATE_WARM_US = 1500000u;                     // 再初期化後の待ち時間
static uint16_t g_arm_gate_hold  = 0;
static uint32_t g_arm_gate_t0    = 0;
static uint8_t  g_ekf_drift_reseed = 1;   // 1 = 次回の ekf_drift_update で差分の基準を取り直す

// アーム時: 四元数を**加速度計から求めた実際の傾き**へ、ジャイロバイアス状態を起動時静止平均へ、
// 共分散を初期化する。
//
// 加速度計と四元数の関係は EKF の観測式そのもの（ekf.cpp observation_equation）:
//   z(0) = 2(q1q3 - q0q2)g = e13·g,  z(1) = 2(q2q3 + q0q1)g = e23·g,  z(2) = (q0²-q1²-q2²+q3²)g = e33·g
// 一方 Euler 抽出は Phi = atan2(e23, e33), Theta = atan2(-e13, hypot(e23,e33)) なので、
// 静止時は Phi = atan2(Ay, Az), Theta = atan2(-Ax, hypot(Ay, Az)) で直接求まる。
// ヨーは磁気観測に任せるため**現在の推定値をそのまま引き継ぐ**（ここで 0 に落とすと磁気残差が
// 跳ね上がり、その収束が roll/pitch を揺らして離陸ゲートが無駄に長引く）。
//
// ⚠ 既存の ekf_reinit() は Xe(4..6) を 0 にするが、それは「バイアスが 0」という誤った主張になる。
//    起動時静止平均 Pbias/Qbias/Rbias を入れるのが正しい（Arm_flag==1 の初期化と同じ）。
// ⚠ Arm_flag==3 の間は sensor_read() が呼ばれないので Ax/Ay/Az は墜落時の値のまま。
//    ここで読み直す。
static void ekf_reinit_on_arm(void)
{
  sensor_read();

  const float an = sqrtf(Ax * Ax + Ay * Ay + Az * Az);
  float phi0 = 0.0f, theta0 = 0.0f;
  if (fabsf(an - GRAV) < 2.0f)          // 静止しているときだけ加速度を信じる
  {
    phi0   = atan2f(Ay, Az);
    theta0 = atan2f(-Ax, sqrtf(Ay * Ay + Az * Az));
  }
  // ヨーは現在の推定を引き継ぐ（四元数が壊れていたら 0）
  float psi0 = 0.0f;
  {
    const float q0 = Xe(0, 0), q1 = Xe(1, 0), q2 = Xe(2, 0), q3 = Xe(3, 0);
    const float e11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    const float e12 = 2.0f * (q1 * q2 + q0 * q3);
    if (isfinite(e11) && isfinite(e12) && (fabsf(e11) + fabsf(e12)) > 1.0e-6f)
      psi0 = atan2f(e12, e11);
  }
  const float cph = cosf(0.5f * phi0),   sph = sinf(0.5f * phi0);
  const float cth = cosf(0.5f * theta0), sth = sinf(0.5f * theta0);
  const float cps = cosf(0.5f * psi0),   sps = sinf(0.5f * psi0);
  Xe << cps * cth * cph + sps * sth * sph,
        cps * cth * sph - sps * sth * cph,
        cps * sth * cph + sps * cth * sph,
        sps * cth * cph - cps * sth * sph,
        Pbias, Qbias, Rbias;
  Xp = Xe;
  Beta << 0.0f, 0.0f, 0.0f;
  P.setIdentity();
  Ekf_drift_phi      = 0.0f;
  Ekf_drift_theta    = 0.0f;
  g_ekf_drift_reseed = 1;
  g_arm_gate_ok      = 0;
  g_arm_gate_hold    = 0;
  g_arm_gate_t0      = time_us_32();
  g_arm_reinit_count++;
  att_trim_reset();   // 2026-09-11: 前の飛行のトリムを引きずらせない
}

static void ekf_bias_reset(void)
{
  Xe(4, 0) = Pbias;  Xe(5, 0) = Qbias;  Xe(6, 0) = Rbias;
  Xp(4, 0) = Pbias;  Xp(5, 0) = Qbias;  Xp(6, 0) = Rbias;
  Ekf_drift_phi = 0.0f;
  Ekf_drift_theta = 0.0f;
  Ekf_bias_reset_count++;
}

// 100Hz、EKF 更新とオイラー角変換の直後に呼ぶ。
static void ekf_drift_update(void)
{
  static uint8_t first = 1;
  static float phi_prev = 0.0f, theta_prev = 0.0f;
  static uint8_t run = 0;
  const float dt = 0.01f;
  if (first || g_ekf_drift_reseed)
  {
    first = 0;
    g_ekf_drift_reseed = 0;
    phi_prev = Phi;
    theta_prev = Theta;
    run = 0;
    return;
  }

  // 姿勢角の実測変化率（±π の折り返しを処理）
  float dphi = Phi - phi_prev;
  if (dphi >  (float)M_PI) dphi -= 2.0f * (float)M_PI;
  if (dphi < -(float)M_PI) dphi += 2.0f * (float)M_PI;
  const float dtheta = Theta - theta_prev;
  phi_prev = Phi; theta_prev = Theta;

  // ジャイロ（NMPC と同じ静止平均基準）から作るオイラー角速度
  const float p = Wp - Pbias, q = Wq - Qbias, r = Wr - Rbias;
  const float sph = sinf(Phi), cph = cosf(Phi);
  const float cth = cosf(Theta);
  if (fabsf(cth) < 0.5f) { run = 0; return; }   // |theta| > 60 deg では tan が暴れるので判定しない
  const float tth = sinf(Theta) / cth;
  const float phi_dot_g   = p + sph * tth * q + cph * tth * r;
  const float theta_dot_g = cph * q - sph * r;

  const float res_phi   = dphi   / dt - phi_dot_g;
  const float res_theta = dtheta / dt - theta_dot_g;
  Ekf_drift_phi   += EKF_DRIFT_LPF_A * (res_phi   - Ekf_drift_phi);
  Ekf_drift_theta += EKF_DRIFT_LPF_A * (res_theta - Ekf_drift_theta);

  const bool over = !(fabsf(Ekf_drift_phi) <= EKF_DRIFT_LIMIT) || !(fabsf(Ekf_drift_theta) <= EKF_DRIFT_LIMIT);
  if (over) { if (++run >= EKF_DRIFT_RUN_N) { ekf_bias_reset(); run = 0; } }
  else run = 0;

  // ---- 離陸ゲート ----
  // 再初期化から 1.5 s 経過し、かつ残差が ±1 deg/s 以内を 1 s continuous で満たしたら開く。
  // 開くまで 400Hz 側がモータを止める。開かないまま飛ばせないのは意図した安全側の挙動。
  if (!g_arm_gate_ok)
  {
    if ((uint32_t)(time_us_32() - g_arm_gate_t0) < EKF_GATE_WARM_US)
    {
      g_arm_gate_hold = 0;
    }
    else if (fabsf(Ekf_drift_phi) < EKF_GATE_LIMIT && fabsf(Ekf_drift_theta) < EKF_GATE_LIMIT)
    {
      if (++g_arm_gate_hold >= EKF_GATE_HOLD_N)
      {
        g_arm_gate_ok = 1;
        g_arm_gate_wait_ms = (time_us_32() - g_arm_gate_t0) / 1000u;
      }
    }
    else
    {
      g_arm_gate_hold = 0;
    }
  }
}

// 四元数と共分散だけを初期値に戻す。Q/R/G は定数なので触らない。
static void ekf_reinit(void)
{
  Xe << 1.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Xp = Xe;
  Beta << 0.0, 0.0, 0.0;
  P.setIdentity();
  Ekf_reset_count++;
}

// 非有限 (NaN/Inf) と、物理的にありえない大きさを弾く。
// NaN は「あらゆる比較が false」なので、!(|x| <= 上限) の形で書けば同時に捕まる。
static bool attitude_state_sane(void)
{
  const float ang_lim  = 2.0f * (float)M_PI;   // |phi|,|theta| が一周を超えたら壊れている
  const float rate_lim = 40.0f;                // [rad/s] ≈ 2290 deg/s。LSM9DS1 の ±2000dps 超
  if (!(fabsf(Phi) <= ang_lim) || !(fabsf(Theta) <= ang_lim)) return false;
  if (!(fabsf(Wp) <= rate_lim) || !(fabsf(Wq) <= rate_lim) || !(fabsf(Wr) <= rate_lim)) return false;
  return true;
}

// ---- ベンチ加振シーケンス ----
// 経過時間で 4 フェーズを進め、6 発の duty を直接書く。NMPC は通さない。
//   (1) t < t_spin                     : 全発 u0        (静止確認・ロータ立ち上げ)
//   (2)(3) < t_spin+t_step             : u0 + kS*delta  (純ヨートルク。合計推力は不変)
//   (4) < t_spin+t_step+t_decay        : 全発 u0        (tau_z = 0。自由減衰)
//   以降                                : 全発 0         (終了)
static void bench_excite_update(void)
{
  // ヨー反トルクの符号。★nmpc_model.cpp の kS と必ず一致させること★
  static const float kS_bench[NUM_MOTOR] = {-1.f, 1.f, 1.f, -1.f, -1.f, 1.f};
  float *d6[NUM_MOTOR] = {&FR_duty, &FL_duty, &MR_duty, &ML_duty, &RR_duty, &RL_duty};

  if (Bench_mode == BENCH_MOTORS_OFF)
  {
    // フェーズ0: モータを回さずログだけ取る。手回しの自由減衰から
    // 軸受摩擦 + 機体空力のベースラインを得る。
    Bench_phase = 1;
    for (int j = 0; j < NUM_MOTOR; ++j) *d6[j] = 0.0f;
    return;
  }

  const float t  = (float)(time_us_32() - Bench_t0_us) * 1.0e-6f;
  const float t1 = Bench_t_spin;
  const float t2 = t1 + Bench_t_step;
  const float t3 = t2 + Bench_t_decay;

  float delta;
  if (t < t1)       { Bench_phase = 1; delta = 0.0f; }
  else if (t < t2)  { Bench_phase = 2; delta = Bench_delta * Bench_sign; }
  else if (t < t3)  { Bench_phase = 3; delta = 0.0f; }
  else
  {
    Bench_phase = 4;
    for (int j = 0; j < NUM_MOTOR; ++j) *d6[j] = 0.0f;
    return;
  }

  for (int j = 0; j < NUM_MOTOR; ++j)
  {
    float u = Bench_u0 + kS_bench[j] * delta;
    if (u < 0.0f) u = 0.0f;
    if (u > nmpc::mpc::U_duty_max) u = nmpc::mpc::U_duty_max;
    *d6[j] = u;
  }
}

// ---- フェイルセーフ / 脱出条件 ----
// 一度成立したらラッチする。目的は「操縦不能になった機体をそれ以上飛ばさない」ことなので、
// 復帰は手動 (再アーム) とする。★解除は loop_400Hz() の Arm_flag 分岐より前で行う★
// (この関数は Arm_flag==2 の枝からしか呼ばれないため、ここに解除を置くと永久に解除できない)。
static void failsafe_check(void)
{
  // 解除は loop_400Hz() の分岐前で行う。ここは Arm_flag==2 の枝からしか呼ばれないので
  // この関数の中に解除処理を置いても実行されない (過去にそれで解除できなくなった)。
  if (Arm_flag != 2)
    return;
  if (Failsafe_flag)
  {
    motor_stop();
    return;
  }

  uint8_t cause = FS_NONE;

  // (1) 受信機のフラグ。0x08 と 0x04 は意味がまったく違うので分けて扱う。
  //     0x08 FAILSAFE  : 受信機が送信機を見失った。本物なので即発動。
  //     0x04 FRAME_LOST: 1 フレーム取りこぼしただけで立つ。電波が少し混めば日常的に起きる。
  //                      単発で空中のモータを永久に切っていたため誤停止していた
  //                      (2026-09-04 0904_v11)。連続本数で判定する。
  if (Rx_sbus_flags & SBUS_FLAG_FAILSAFE)
    cause = FS_RX_FAILSAFE;
  else if (Rx_framelost_run >= FS_FRAMELOST_RUN_LIMIT)
    cause = FS_RX_FRAMELOST;

  // (2) 受信途絶。フレームが来なくなると Chdata は最後の値で固まるので、時間で見る。
  if (cause == FS_NONE)
  {
    const uint32_t now = time_us_32();
    if (Radio_last_frame_us != 0 && (uint32_t)(now - Radio_last_frame_us) > FS_RX_TIMEOUT_US)
      cause = FS_RX_TIMEOUT;
  }

  // (3)(4) 姿勢・角速度の脱出条件。バイアスを引いた値で判定する。
  if (cause == FS_NONE)
  {
    const float phi_e   = Phi - Phi_bias;
    const float theta_e = Theta - Theta_bias;
    // ベンチ加振では機体を上下逆さに吊るため |phi| が 180° 付近になり、
    // 姿勢の脱出条件が常時成立してしまう。機体は治具で拘束されていて飛び去らないので
    // 姿勢判定だけ外す。角速度・過大G・受信途絶はそのまま生かす。
    if (Bench_mode == BENCH_OFF &&
        (fabsf(phi_e) > FS_ATTITUDE_LIMIT || fabsf(theta_e) > FS_ATTITUDE_LIMIT))
      cause = FS_ATTITUDE;
    else if (fabsf(Wp - Pbias) > FS_RATE_LIMIT ||
             fabsf(Wq - Qbias) > FS_RATE_LIMIT ||
             fabsf(Wr - Rbias) > FS_RATE_LIMIT)
      cause = FS_RATE;
  }

  // (5) 過大 G (sensor_read が立てる)
  if (cause == FS_NONE && OverG_flag)
    cause = FS_OVERG;

  if (cause != FS_NONE)
  {
    Failsafe_flag = 1;
    Failsafe_cause = cause;
    motor_stop();
  }
}

void motor_stop(void)
{
  set_duty_fr(0.0);
  set_duty_fl(0.0);
  set_duty_mr(0.0);
  set_duty_ml(0.0);
  set_duty_rr(0.0);
  set_duty_rl(0.0);
}

// 高度制御の関数宣言
void lotate_altitude_init(float Theta, float Psi, float Phi)
{
  lotate_mat(0, 0) = cos(Theta) * cos(Psi);
  lotate_mat(0, 1) = cos(Theta) * sin(Psi);
  lotate_mat(0, 2) = -sin(Theta);
  lotate_mat(1, 0) = (sin(Phi) * sin(Theta) * cos(Psi)) - (cos(Phi) * sin(Psi));
  lotate_mat(1, 1) = (sin(Phi) * sin(Theta) * sin(Psi)) + (cos(Phi) * cos(Psi));
  lotate_mat(1, 2) = sin(Phi) * cos(Theta);
  lotate_mat(2, 0) = (cos(Phi) * sin(Theta) * cos(Psi)) + (sin(Phi) * sin(Psi));
  lotate_mat(2, 1) = (cos(Phi) * sin(Theta) * sin(Psi)) - (sin(Phi) * cos(Psi));
  lotate_mat(2, 2) = cos(Phi) * cos(Theta);
}

float lotate_altitude(float l_distance)
{
  // distance_mat(0,0) = 0;
  // distance_mat(0,1) = 0;
  distance_mat(0, 2) = l_distance;
  f_distance_mat = distance_mat * lotate_mat;
  f_distance = f_distance_mat(0, 0);
  f_distance2 = f_distance_mat(0, 1);
  f_distance3 = f_distance_mat(0, 2);

  return f_distance3;
}

// ホバリング
void Hovering(void)
{
  // 実験なので4秒
  // 本番はゴールを見つけたら着陸モード
  //  if (hove_time < 10)
  //  {
  //    input = alt_PID(ideal);
  //    T_ref = T_stick + (input);
  //    hove_time = hove_time + 0.01;
  //  }
  //  else{
  //    flying_mode = 3;
  //    //Auto_landing();
  //  }
  //  if (gap_number >=50){
  //    landing_counter = 1;
  //  }
  input = alt_PID(ideal);
  T_ref = T_stick + (input);
}


void Merker_Hovering(void){
  //高度制御テスト用のコード
  if(Flight_mode == MODE_HOVERING){
    // auto_mode =1;
    if(first_run){
      // ideal = mu_Yn_est(1,0);
      ideal = Kalman_alt;
      T_stick = 0.7 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
      // auto_mode_count = 1;
      first_run = false;
    }
    if (count_up == 10) {
      input = alt_PID(ideal);
      T_ref = T_stick + (input);
      count_up = 0;
    }
    count_up = count_up + 1;
    printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
  }
  else{
    first_run = true;
    // auto_mode =0;
    // auto_mode_count = 0;
    T_ref = 0.7 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
    // printf("T_ref:%9.6f ac:%9.6f\n",T_ref,auto_mode_count);
    integral = 0;
    integral_v = 0;
    differential = 0;
    differential_v = 0;
  }

  // plotnum = plotnum+1;
  // if (plotnum==65000){
  //   plotnum = 0;
  // }

  // if (auto_mode ==1){
  //   // // 目標値との誤差
  //   // TOL_x_err = 0.008086 * (TOL_x_ref - TOL_x_diff);
  //   // TOL_y_err = 0.01 * (TOL_y_ref - TOL_y_diff);

  //   // Pref = phi_pid.update(TOL_x_err);
  //   // Qref = theta_pid.update(TOL_y_err);
  //   if(auto_mode_count == 0){
  //     auto_mode_count = 1;
  //     ideal = mu_Yn_est(1,0);
  //     T_stick = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
  //   }
  //   if (count_up == 10){
  //     input = alt_PID(ideal);
  //     T_ref = T_stick + (input);
  //     printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
  //     // printf("%9.6f %9.6f %9.6f %9.6f %9.6f f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input);
  //     // Hovering();
  //     count_up = 0;
  //   }
  //   count_up = count_up + 1;
  // }
  // printf("%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %d %9.6f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input,flying_mode,count_up,auto_mode_count);
}

void servo_control(void)
{ 
  if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2)
    payload_relese();
  if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2)
    payload_hook();
  // if (Flight_mode == NORMAL){
  //   if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //     payload_relese();
  //   if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2 && Flight_mode == NORMAL)
  //     payload_hook();
  // }
  // else if(Flight_mode == REDCIRCLE){
  //   if (Chdata[SERVO] > (SERVO_MAX + SERVO_MIN) / 2)
  //     payload_relese();
  //   // if (Chdata[SERVO] < (SERVO_MAX + SERVO_MIN) / 2 && )
  //   //   payload_hook();
  // }
}

void rate_control(void)
{
  float p_rate, q_rate, r_rate;
  float p_ref, q_ref, r_ref;
  float p_err, q_err, r_err;

  // Read Sensor Value
  sensor_read();

  // Mode SW
  // Chdata[MODE_SW]=1000;//本番はコメントにする
  // if (Chdata[MODE_SW]>1241)

  // if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  // if ((Chdata[REDCIRCLE] < 200) &&(Chdata[LOG] < 200) && (Chdata[HOVERING] < 200)&& i2c_connect == 1)
  // ⚠ ch5 (radio.hpp の FTC) は耐故障スイッチへ転用したので、Flight_mode の判定から外した。
  //    外さないと FTC スイッチを倒すたびに Flight_mode まで変わる。
  // 2026-09-10: 判定から Chdata[LOG] を外した。LOG スイッチ ON の間はどの分岐にも入らず
  // Flight_mode が更新されなかったため、直前に FTC を触って MODE_FTC が残ると
  // led_control() がどの分岐にも該当せず消灯していた（0910_v7 の「ログモードで arm すると
  // LED が点かない、LOG を OFF→ON で直る」）。ログ記録は logging() 側で独立に判定しているので
  // 飛行モードに LOG を絡める理由はない。
  // 2026-09-11: HOVERING スイッチによるモード分岐を削除した（運用しているのは NORMAL と FTC だけ）。
  // 旧コードは  if (Chdata[HOVERING] < 200) NORMAL / else if (> 500) MODE_HOVERING  の 2 分岐で、
  // **200-500 のデッドゾーンでは Flight_mode を一切書かなかった**。
  // 下の FTC ブロックは `if (Ftc_mode) Flight_mode = MODE_FTC;` と書くだけで、OFF 時の復帰を
  // 「ここが毎周期 NORMAL を書き直すこと」に依存している。スイッチがデッドゾーンに居るとその前提が
  // 崩れ、FTC を切っても MODE_FTC が居座る → led_control() の
  //   Arm_flag==2 && Ftc_mode / == NORMAL / == MODE_HOVERING
  // のどの枝にも当たらず（else が無い）**WS2812 が一度も書かれず直前の色で固まる**。
  // 0911_v7 の「ディスアームしても LED の色が変わらない」の候補機構。
  // 詳細: docs/ai/decisions/20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md
  // 無条件に NORMAL を書くことで、到達し得る Flight_mode は NORMAL と MODE_FTC だけになり、
  // led_control() の枝で必ず拾われる（Ftc_mode==1 は FTC の枝、0 は NORMAL の枝）。
  Flight_mode = NORMAL;
  Red_flag = 0;
  Rocking_timer = 0.0;

  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] > 500) && (Chdata[ROCKING] < 200) && i2c_connect == 0)
  // {
  //   Flight_mode = NORMAL;
  //   Red_flag = 0;
  //   Rocking_timer = 0.0;
  // }

  // else if ( (Chdata[REDCIRCLE] < 200)&&(Chdata[LOG] < 200) && (Chdata[HOVERING] > 500)&& i2c_connect == 1)
  // 2026-09-11: 削除（MODE_HOVERING は運用していない）。
  //   else if (Chdata[HOVERING] > 500) { Flight_mode = MODE_HOVERING; Red_flag = 0; }
  // ⚠ 復活させるなら、デッドゾーンを作らない単一しきい値にすること。
  //   led_control() には MODE_HOVERING の枝が残してあるので表示は動く。

  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] < 200) && (Chdata[LINETRACE] > 500) && (Chdata[ROCKING] < 200) && i2c_connect == 1)
  // {
    // Flight_mode = LINETRACE;
    // Red_flag = 0;
    // Rocking_timer = 0.0;
  // }
  // 赤丸検知モード
  // else if ((Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[HOVERING] < 200))
  // 旧 REDCIRCLE モードは削除

  // ---- 耐故障(FTC)モード: ch5 で対象モータを故障扱いにする ----
  // 200-500 は不感帯 (前回値保持)。既存のモード判定と同じ流儀。
  // スイッチ ON で Flight_mode も MODE_FTC にする。OFF 時の復帰は上の NORMAL/HOVERING 分岐が
  // 毎周期 Flight_mode を書き直すことで成立する（LOG スイッチに依存しなくなった）。
  {
    const uint8_t ftc_prev = Ftc_mode;
    if (Chdata[FTC] > 500)
      Ftc_mode = 1;
    else if (Chdata[FTC] < 200)
      Ftc_mode = 0;

    if (Ftc_mode != ftc_prev)
    {
      // η を作り直す。故障指定が MOTOR_NONE なら何も止めない。
      for (int j = 0; j < NUM_MOTOR; ++j)
        Motor_eta[j] = 1.0f;
      if (Ftc_mode && Ftc_target_motor >= 0 && Ftc_target_motor < NUM_MOTOR)
        Motor_eta[Ftc_target_motor] = 0.0f;
      g_motor_eta_dirty = 1;   // core1 (angle_control) 側で NMPC に反映する
    }
    if (Ftc_mode)
      Flight_mode = MODE_FTC;
  }
  // if((Chdata[REDCIRCLE] > 500)){

  // }
  // printf("%d\n",(Chdata[REDCIRCLE]));
  

  // printf("T_ref_coe :%f \n",T_ref_coe);

  // else if ((Chdata[SERVO] < 200) && (Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  // {
  //   Flight_mode = REDCIRCLE;
  //   release_flag = 0;
  //   Rocking_timer = 0.0;
  // }

  // else if ((Chdata[SERVO] > 500) && (Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[ROCKING] < 200))
  // {
  //   Flight_mode = SERVO;
  //   Rocking_timer = 0.0;
  // }

  // Get Bias
  // Pbias = Xe(4, 0);
  // Qbias = Xe(5, 0);
  // Rbias = Xe(6, 0);

  // Control angle velocity
  p_rate = Wp - Pbias;
  q_rate = Wq - Qbias;
  r_rate = Wr - Rbias;

  // Merker_Hovering();

  // Get reference
  p_ref = Pref;
  q_ref = Qref;
  r_ref = Rref;

  // if (Flight_mode != LINETRACE)
  // if (Flight_mode != HOVERING)
#if defined(USE_NMPC) && NMPC_BENCH_MODE
  // 机上検証でもスロットル CH3 に追従 (Arm_flag==2 時も同式)
  T_ref = 0.2f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
#else
    // Benzen_new と同一係数 (0.5 / T_ref_coe=0.6)。変えたら必ず実測し直すこと。
    T_ref = 0.5f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
#endif

  // Merker_Hovering();

  // //高度制御テスト用のコード
  // if(Flight_mode == HOVERING){
  //   auto_mode =1;
  // }
  // else{
  //   auto_mode =0;
  //   auto_mode_count = 0;
  //   T_ref = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  //   // printf("T_ref:%9.6f ac:%9.6f\n",T_ref,auto_mode_count);
  // }

  // plotnum = plotnum+1;
  // if (plotnum==65000){
  //   plotnum = 0;
  // }
  // if (auto_mode ==1){
  //   // // 目標値との誤差
  //   // TOL_x_err = 0.008086 * (TOL_x_ref - TOL_x_diff);
  //   // TOL_y_err = 0.01 * (TOL_y_ref - TOL_y_diff);

  //   // Pref = phi_pid.update(TOL_x_err);
  //   // Qref = theta_pid.update(TOL_y_err);
  //   if(auto_mode_count == 0){
  //     auto_mode_count = 1;
  //     // flying_mode = 1;
  //     ideal = mu_Yn_est(1,0);
  //     T_stick = 0.5 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN); 
  //   }
  //   if (count_up == 10){
  //     input = alt_PID(ideal);
  //     T_ref = T_stick + (input);
  //     printf("%d %9.6f %9.6f %9.6f %9.6f %9.6f\n",plotnum,ideal,mu_Yn_est(1,0),mu_Yn_est(0,0),input,T_ref);
  //     // printf("%9.6f %9.6f %9.6f %9.6f %9.6f f\n",mu_Yn_est(1,0),ideal,T_ref,T_stick,input);
  //     // Hovering();
  //     count_up = 0;
  //   }
  //   count_up = count_up + 1;
  // }

  // //高度制御テスト用のコード
  // if(Chdata[SERVO] > 500){
  //   auto_mode =1;
  // }
  // else{
  //   auto_mode =0;
  //   auto_mode_count = 0;
  //   T_ref = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  // }

  // if (auto_mode ==1){
  //   if (count_up >= 10){
  //     count_up = 0;
  //     if(auto_mode_count ==0){
  //       auto_mode_count = 1;
  //       flying_mode = 1;
  //       ideal = Kalman_alt;
  //       T_stick = 0.6 * BATTERY_VOLTAGE*(float)(Chdata[2]-CH3MIN)/(CH3MAX-CH3MIN);
  //     }
  //     //printf("Auto Kalman_alt : %9.6f\n",Kalman_alt);
  //     //Auto_fly();
  //     //Auto_takeoff();
  //     //Auto_landing();
  //     Hovering();
  //   }
  //   count_up += 1;
  // }

#ifndef USE_NMPC
  // Error
  p_err = p_ref - p_rate;
  q_err = q_ref - q_rate;
  r_err = r_ref - r_rate;

  // PID
  P_com = p_pid.update(p_err);
  Q_com = q_pid.update(q_err);
  R_com = r_pid.update(r_err);

  // saturation P_com
  if (P_com >= 3.7)
  {
    P_com = 3.7;
  }
  else if (P_com <= -3.7)
  {
    P_com = -3.7;
  }

  // saturation R_com
  if (R_com >= 3.7)
  {
    R_com = 3.7;
  }
  else if (R_com <= -3.7)
  {
    R_com = -3.7;
  }

  // saturation Q_com
  if (Q_com >= 3.7)
  {
    Q_com = 3.7;
  }
  else if (Q_com <= -3.7)
  {
    Q_com = -3.7;
  }

  // Motor Control(mixing)
  //  1250/7.4=112.6
  //  1/7.4=0.1351(2セル時)
  // 1/11.1 = 0.0901（3セル時）
  // FR_duty = (T_ref + (-P_com + Q_com - R_com) * 0.25) * 0.1351;
  // FL_duty = (T_ref + (P_com + Q_com + R_com) * 0.25) * 0.1351;
  // RR_duty = (T_ref + (-P_com - Q_com + R_com) * 0.25) * 0.1351;
  // RL_duty = (T_ref + (P_com - Q_com - R_com) * 0.25) * 0.1351;

  // FR_duty = (T_ref*0.16666667 + (-0.16666667*P_com + 0.28867513*Q_com - 0.16666667*R_com) * 0.25) * 0.0901;
  // FL_duty = (T_ref*0.16666667 + (0.16666667*P_com  + 0.28867513*Q_com + 0.16666667*R_com) * 0.25) * 0.0901;
  // MR_duty = (T_ref*0.16666667 + (-0.33333333*P_com + 0*Q_com          + 0.16666667*R_com) * 0.25) * 0.0901;
  // ML_duty = (T_ref*0.16666667 + (0.33333333*P_com  + 0*Q_com          - 0.16666667*R_com) * 0.25) * 0.0901;
  // RR_duty = (T_ref*0.16666667 + (-0.16666667*P_com - 0.28867513*Q_com - 0.16666667*R_com) * 0.25) * 0.0901;
  // RL_duty = (T_ref*0.16666667 + (0.16666667*P_com  - 0.28867513*Q_com + 0.16666667*R_com) * 0.25) * 0.0901;

  // FR_duty = (T_ref*1 + (-0.48*P_com + 0.84*Q_com - 0.48*R_com) * 0.25) * 0.0901;
  // FL_duty = (T_ref*1 + (0.48*P_com  + 0.84*Q_com + 0.48*R_com) * 0.25) * 0.0901;
  // MR_duty = (T_ref*1 + (-0.96*P_com + 0*Q_com   + 0.48*R_com) * 0.25) * 0.0901;
  // ML_duty = (T_ref*1 + (0.96*P_com  + 0*Q_com   - 0.48*R_com) * 0.25) * 0.0901;
  // RR_duty = (T_ref*1 + (-0.48*P_com - 0.84*Q_com - 0.48*R_com) * 0.25) * 0.0901;
  // RL_duty = (T_ref*1 + (0.48*P_com  - 0.84*Q_com + 0.48*R_com) * 0.25) * 0.0901;

  float FR_duty = (T_ref*T_ref_coe + (-1.7655 *P_com + 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  float MR_duty = (T_ref*T_ref_coe + (-3.5311*P_com  + 0.0*Q_com     + 1.7655 *R_com)  ) ;
  float RR_duty = (T_ref*T_ref_coe + (-1.7655*P_com  - 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  float RL_duty = (T_ref*T_ref_coe + (1.7655*P_com   - 3.0562 *Q_com + 1.7655 *R_com)  ) ;
  float ML_duty = (T_ref*T_ref_coe + (3.5311*P_com   - 0.0*Q_com     - 1.7655 *R_com)  ) ;
  float FL_duty = (T_ref*T_ref_coe + (1.7655 *P_com  + 3.0562 *Q_com + 1.7655 *R_com)  ) ;

  // float FR_duty = (T_ref*0.1089 + (-1.7655 *P_com + 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  // float MR_duty = (T_ref*0.1089 + (-3.5311*P_com  + 0.0*Q_com     + 1.7655 *R_com)  ) ;
  // float RR_duty = (T_ref*0.1089 + (-1.7655*P_com  - 3.0562 *Q_com - 1.7655 *R_com)  ) ;
  // float RL_duty = (T_ref*0.1089 + (1.7655*P_com   - 3.0562 *Q_com + 1.7655 *R_com)  ) ;
  // float ML_duty = (T_ref*0.1089 + (3.5311*P_com   - 0.0*Q_com     - 1.7655 *R_com)  ) ;
  // float FL_duty = (T_ref*0.1089 + (1.7655 *P_com  + 3.0562 *Q_com + 1.7655 *R_com)  ) ;
 
  // FR_duty = (T_ref*1 + (-P_com + Q_com - R_com) * 0.25) * 0.0901;
  // FL_duty = (T_ref*1 + (P_com  + Q_com + R_com) * 0.25) * 0.0901;
  // MR_duty = (T_ref*1 + (-P_com + 0*Q_com +R_com) * 0.25) * 0.0901;
  // ML_duty = (T_ref*1 + (P_com  + 0*Q_com -R_com) * 0.25) * 0.0901;
  // RR_duty = (T_ref*1 + (-P_com - Q_com - R_com) * 0.25) * 0.0901;
  // RL_duty = (T_ref*1 + (P_com  -Q_com + R_com) * 0.25) * 0.0901;

  float minimum_duty = 0.1;
  const float maximum_duty = 0.95;
  minimum_duty = Disable_duty;

  if (FR_duty < minimum_duty)
    FR_duty = minimum_duty;
  if (FR_duty > maximum_duty)
    FR_duty = maximum_duty;

  if (FL_duty < minimum_duty)
    FL_duty = minimum_duty;
  if (FL_duty > maximum_duty)
    FL_duty = maximum_duty;

  if (MR_duty < minimum_duty)
    MR_duty = minimum_duty;
  if (MR_duty > maximum_duty)
    MR_duty = maximum_duty;

  if (ML_duty < minimum_duty)
    ML_duty = minimum_duty;
  if (ML_duty > maximum_duty)
    ML_duty = maximum_duty;

  if (RR_duty < minimum_duty)
    RR_duty = minimum_duty;
  if (RR_duty > maximum_duty)
    RR_duty = maximum_duty;

  if (RL_duty < minimum_duty)
    RL_duty = minimum_duty;
  if (RL_duty > maximum_duty)
    RL_duty = maximum_duty;

  // Duty set
  const float common_duty = T_ref * T_ref_coe;  // 姿勢ゼロ時の PWM
  if (common_duty < Disable_duty)
  {
    motor_stop();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();

    Pref = 0.0;
    Qref = 0.0;
    Rref = 0.0;
#ifndef USE_NMPC
    Aileron_center = Chdata[3];
    Elevator_center = Chdata[1];
    Rudder_center = Chdata[0];
#endif
    Phi_bias = Phi;
    Theta_bias = Theta;
    Psi_bias = Psi;
  }
  else
  {
    if (OverG_flag == 0)
    {
      set_duty_fr(FR_duty);//FR_duty
      set_duty_fl(FL_duty);//FL_duty
      set_duty_mr(MR_duty);//MR_duty
      set_duty_ml(ML_duty);//ML_duty
      set_duty_rr(RR_duty);//RR_duty
      set_duty_rl(RL_duty);//RL_duty
    }

    else
      motor_stop();
    // printf("%12.5f %12.5f %12.5f\n",p_rate, q_rate, r_rate);
  }
#else
  // ===== NMPC モード =====
  // モータ duty は 100Hz の angle_control(NMPC) がグローバル FR_duty..RL_duty に
  // 格納済。ここでは clamp + フェイルセーフ + safety(Disable/OverG) を適用して
  // 400Hz で ZOH 出力する(既存セーフティ構造を維持)。
  {
    float minimum_duty = Disable_duty;
    const float maximum_duty = nmpc::mpc::U_duty_max;
    float *duties[6] = {&FR_duty, &FL_duty, &MR_duty, &ML_duty, &RR_duty, &RL_duty};
    for (int i = 0; i < 6; ++i)
    {
      // NaN フェイルセーフ (自己比較が false なら NaN)
      if (*duties[i] != *duties[i])
        *duties[i] = minimum_duty;
      // 故障指定のモータは下限を適用せず 0 にする (二重ガード)。
      // これを入れないと minimum_duty=0.1 で回り続ける。
      if (Motor_eta[i] <= 0.5f)
      {
        *duties[i] = 0.0f;
        continue;
      }
      // ベンチ加振モードは指令をそのまま出す。minimum_duty=0.1 で持ち上げると
      // フェーズ0 の「モータ停止のままログ」も u0 の正確な再現もできなくなる。
      if (Bench_mode != BENCH_OFF)
      {
        if (*duties[i] < 0.0f) *duties[i] = 0.0f;
        if (*duties[i] > maximum_duty) *duties[i] = maximum_duty;
        continue;
      }
      if (*duties[i] < minimum_duty)
        *duties[i] = minimum_duty;
      if (*duties[i] > maximum_duty)  // +Inf もここで丸まる
        *duties[i] = maximum_duty;
    }

    const float common_duty = T_ref * T_ref_coe;  // 姿勢ゼロ時の PWM
    if (Failsafe_flag)
    {
      motor_stop();   // フェイルセーフ成立中は何があっても出力しない
    }
    else if (!g_arm_gate_ok && Bench_mode == BENCH_OFF)
    {
      // 離陸ゲート: 推定器が落ち着くまでスロットルを無視する（LED = 赤の速い点滅）。
      // ベンチ加振モードは姿勢推定を使わないので対象外。
      motor_stop();
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      Phi_bias = Phi;
      Theta_bias = Theta;
      Psi_bias = Psi;
    }
    else if (common_duty < Disable_duty)
    {
      motor_stop();
      // NMPC warm-start 破棄は angle_control の !nmpc_run で行う。
      // ここで毎 400Hz 呼ぶと TR1/SR2 が育たずフル再構築が連続する。
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      if (Arm_flag != 2)
      {
        Aileron_center = Chdata[3];
        Elevator_center = Chdata[1];
        Rudder_center = Chdata[0];
      }
      Phi_bias = Phi;
      Theta_bias = Theta;
      Psi_bias = Psi;
    }
    else
    {
      if (OverG_flag == 0)
      {
        
        set_duty_fr(FR_duty);
        set_duty_fl(FL_duty);
        set_duty_mr(MR_duty);
        set_duty_ml(ML_duty);
        set_duty_rr(RR_duty);
        set_duty_rl(RL_duty);
      }
      else
        motor_stop();
    }
  }
#endif

  // printf("\n");

  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, fr_duty, fl_duty, rr_duty, rl_duty, p_rate, q_rate, r_rate);
  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, p_com, q_com, r_com, p_ref, q_ref, r_ref);
  // printf("%12.5f %12.5f %12.5f %12.5f %12.5f %12.5f %12.5f\n",
  //     Elapsed_time, Phi, Theta, Psi, Phi_bias, Theta_bias, Psi_bias);
  // Elapsed_time = Elapsed_time + 0.0025;
  // Logging
  // logging();
}

void angle_control(void)
{
  float phi_err, theta_err, psi_err;
  float q0, q1, q2, q3;
  float e23, e33, e13, e11, e12;
  while (1)
  {
    sem_acquire_blocking(&sem); // 時間統制
    sem_reset(&sem, 0);
    S_time2 = time_us_32();

#ifdef USE_NMPC
    if (g_nmpc_inited)
        nmpc::nmpc_cost_apply_pending(g_nmpc_ctx);
#endif

    kalman_filter(); // 100Hzで計算
    q0 = Xe(0, 0);
    q1 = Xe(1, 0);
    q2 = Xe(2, 0);
    q3 = Xe(3, 0);
    e11 = q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3;
    e12 = 2 * (q1 * q2 + q0 * q3);
    e13 = 2 * (q1 * q3 - q0 * q2);
    e23 = 2 * (q2 * q3 + q0 * q1);
    e33 = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;
    Phi = atan2(e23, e33);
    Theta = atan2(-e13, sqrt(e23 * e23 + e33 * e33));
    Psi = atan2(e12, e11);
    Psi = Xn_est_3;
    ekf_drift_update();   // EKF バイアス発散の検知（姿勢角とジャイロの整合）

    // Get angle ref (manual flight)
    if (1)
    {
      // // Rockingwing
      // if (Flight_mode != ROCKING)
      // {
      //   // Phi_ref   =  Phi_trim + 0.3 *M_PI*(float)(Chdata[3] - (CH4MAX+CH4MIN)*0.5)*2/(CH4MAX-CH4MIN);
      // }
      // else if (Flight_mode == ROCKING)
      // {
      //   Phi_ref = rocking_wings(Phi_ref);
      // }

      // if (Flight_mode == LINETRACE && i2c_connect == 1)
      if (Flight_mode == 100 && i2c_connect == 1)
      {
        // // auto_mode_count = 1;
        // psi_pid.set_parameter(10.0, 1000, 0.01, 0.125, 0.01);
        // if (Linetrace_counter_for_control > 3)
        // {
        //   linetrace();
        //   Linetrace_counter_for_control = 0;
        // }
        // Linetrace_counter_for_control++;

        // if (auto_mode_count == 1) {
        //   auto_mode_count = 0;
        //   ideal = Kalman_alt;
        //   T_stick = 0.6 * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
        // }
        // Hovering();
      }
      else
      {
        update_attitude_refs_from_sticks();
        Psi = 0.0;
        // Linetrace_counter = 0;
        psi_pid.set_parameter(0, 100000, 0.01, 0.125, 0.01);
        // auto_mode_count = 1;
        Range_filter.reset();
        Angle_filter.reset();
        Velocity_filter.reset();
      }
    }

#ifndef USE_NMPC
    // PID Control
    if (T_ref * T_ref_coe < Flight_duty)
    {
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      phi_pid.reset();
      theta_pid.reset();
      psi_pid.reset();
      Aileron_center = Chdata[3];
      Elevator_center = Chdata[1];
      Rudder_center = Chdata[0];
      /////////////////////////////////////
      Phi_bias = Phi;
      Theta_bias = Theta;
      Psi_bias = Psi;
      /////////////////////////////////////
    }
    else
    {
      phi_err = Phi_ref - (Phi - Phi_bias);
      theta_err = Theta_ref - (Theta - Theta_bias);
      psi_err = Psi_ref - (Psi - Psi_bias);

      // Pref = Phi_ref;//phi_pid.update(phi_err);
      // Qref = Theta_ref;//theta_pid.update(theta_err);
      // Rref = Psi_ref; // psi_pid.update(psi_err);
      Pref = phi_pid.update(phi_err);
      Qref = theta_pid.update(theta_err);
      Rref = psi_pid.update(psi_err);
      if (Flight_mode != LINETRACE)
      {
        Rref = Psi_ref;
      }
      // else if (Flight_mode == LINETRACE)
      else if (Flight_mode == 100)
      {
        Rref = psi_pid.update(psi_err);
      }
    }

    // saturation Rref
    if (Rref >= (rate_limit * pi / 180))
    {
      Rref = rate_limit * pi / 180;
    }
    else if (Rref <= -(rate_limit * pi / 180))
    {
      Rref = -(rate_limit * pi / 180);
    }

    // saturation Pref
    if (Pref >= (rate_limit * pi / 180))
    {
      Pref = rate_limit * pi / 180;
    }
    else if (Pref <= -(rate_limit * pi / 180))
    {
      Pref = -(rate_limit * pi / 180);
    }

    // saturation Qref
    if (Qref >= (rate_limit * pi / 180))
    {
      Qref = rate_limit * pi / 180;
    }
    else if (Qref <= -(rate_limit * pi / 180))
    {
      Qref = -(rate_limit * pi / 180);
    }
#else
    // ===== 姿勢 NMPC (加算合成: duty = u_hover + Δu) =====
    bool nmpc_run = false;
    float u_hover_snap = 0.0f;

#if NMPC_BENCH_MODE
    // 机上検証: Arm_flag==2 (飛行準備完了) なら常に NMPC 実行
    nmpc_run = (Arm_flag == 2);
#else
    nmpc_run = (T_ref * T_ref_coe >= Flight_duty);
#endif
    // 離陸ゲート中は 400Hz 側がモータを止めているので、NMPC も止める。
    // 動かしたままにすると warm-start と遅れ補償の duty 履歴が「出していない指令」で育ち、
    // ゲートが開いた瞬間に過渡が出る。
    if (!g_arm_gate_ok && Bench_mode == BENCH_OFF)
      nmpc_run = false;

    if (Bench_mode != BENCH_OFF)
    {
      // ===== ベンチ加振モード: NMPC を通さず duty を直接書く =====
      // スロットルがデッドマンスイッチを兼ねる。下げている間はシーケンスを頭出しする。
      if (T_ref * T_ref_coe < Flight_duty)
      {
        Bench_phase = 0;
        Bench_t0_us = time_us_32();
        FR_duty = FL_duty = MR_duty = ML_duty = RR_duty = RL_duty = 0.0f;
        Phi_bias = Phi;
        Theta_bias = Theta;
        Psi_bias = Psi;
      }
      else
      {
        bench_excite_update();
      }
      // NMPC は動かさないので warm-start は捨てる (復帰時にフル再構築させる)
      if (g_nmpc_was_running)
        nmpc_reset_request();
      g_nmpc_was_running = false;
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      g_nmpc_iter_us = 0;
    }
    else if (!nmpc_run)
    {
      att_trim_reset();   // 2026-09-11: 非飛行中は積分を溜めない
      // 非武装/低スロットル: NMPC 停止、バイアス更新、duty は baseline(=Disable_duty)
      if (g_nmpc_was_running)
        nmpc_reset_request();
      g_nmpc_was_running = false;
      if (Arm_flag != 2)
      {
        Aileron_center = Chdata[3];
        Elevator_center = Chdata[1];
        Rudder_center = Chdata[0];
      }
      Phi_bias = Phi;
      Theta_bias = Theta;
      Psi_bias = Psi;
      FR_duty = FL_duty = MR_duty = ML_duty = RR_duty = RL_duty = Disable_duty;
      {
        float *d6[nmpc::mpc::NU] = {&FR_duty, &FL_duty, &MR_duty, &ML_duty, &RR_duty, &RL_duty};
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          if (Motor_eta[j] <= 0.5f) *d6[j] = 0.0f;
      }
      duty_hist_push();   // 遅れ補償用: 停止中も履歴を更新（再開時に古い値を使わない）
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      g_nmpc_iter_us = 0;
    }
    else if (!attitude_state_sane())
    {
      // ---- 安全弁 3 ----
      // 姿勢推定が壊れた状態を NMPC に渡すと NaN が出て duty が固着する。
      // EKF を初期化し、この周期は baseline duty を出して次周期からやり直す。
      ekf_reinit();
      if (g_nmpc_was_running)
        nmpc_reset_request();
      g_nmpc_was_running = false;
      FR_duty = FL_duty = MR_duty = ML_duty = RR_duty = RL_duty = Disable_duty;
      {
        float *d6[nmpc::mpc::NU] = {&FR_duty, &FL_duty, &MR_duty, &ML_duty, &RR_duty, &RL_duty};
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          if (Motor_eta[j] <= 0.5f) *d6[j] = 0.0f;
      }
      Pref = 0.0;
      Qref = 0.0;
      Rref = 0.0;
      g_nmpc_iter_us = 0;
    }
    else
    {
      g_nmpc_was_running = true;
      // 現在状態 x0 = [phi, theta, p, q, r]
      nmpc::StateVec x0;
      x0(0) = Phi - Phi_bias;
      x0(1) = Theta - Theta_bias;
      x0(2) = Wp - Pbias;
      x0(3) = Wq - Qbias;
      x0(4) = Wr - Rbias;

      // ---- 遅れ補償: 直近に出した duty で x0 を前方予測 ----
      // 実効遅れ 40-50 ms の間に効く指令は既に出してあるので、その分の運動を先回りする。
      if (nmpc::rti::delay_comp_steps > 0)
      {
        nmpc::InputVec hist[G_DUTY_HIST_N];
        const int n = nmpc::rti::delay_comp_steps < G_DUTY_HIST_N ? nmpc::rti::delay_comp_steps : G_DUTY_HIST_N;
        for (int i = 0; i < n; ++i)
        {
          const int idx = (g_duty_hist_idx + G_DUTY_HIST_N - n + i) % G_DUTY_HIST_N;   // 古い順
          for (int j = 0; j < nmpc::mpc::NU; ++j) hist[i](j) = g_duty_hist[idx][j];
        }
        nmpc::StateVec x_pred;
        if (nmpc::nmpc_delay_predict(x0, hist, n, 0.01f, x_pred))
          x0 = x_pred;
        else
          g_delay_pred_nonfinite++;
      }

      // 参照: 姿勢角はスティック、p/q 目標 0、r 目標はヨー rate (Psi_ref)
      nmpc::StateVec x_ref;
      x_ref(0) = Phi_ref;
      x_ref(1) = Theta_ref;
      x_ref(2) = 0.0f;
      x_ref(3) = 0.0f;
      x_ref(4) = Psi_ref;

      // 耐故障: η が変わっていたら NMPC の box 制約を張り直す。
      // Hessian は box に依存しないのでフル再構築(34ms)は起きない。
      if (g_motor_eta_dirty)
      {
        g_motor_eta_dirty = 0;
        nmpc::nmpc_set_motor_eta(g_nmpc_ctx, Motor_eta);
      }

      // ホバー baseline duty (既存ミキサと同一スケール: T_ref * T_ref_coe)
      float u_hover = T_ref * T_ref_coe;
      if (u_hover < 0.0f)
        u_hover = 0.0f;
      if (u_hover > nmpc::mpc::U_duty_max)
        u_hover = nmpc::mpc::U_duty_max;
      u_hover_snap = u_hover;

      // 参照スタック (static で BSS に確保しスタック消費を回避)
      static nmpc::XStackVec X_ref_stack;
      static nmpc::UStackVec U_ref_stack;
      // U_ref シフト (repository_rules §3)。健全時は全 0。
      // 故障時は「止まる分を残りで肩代わりする」動作点を参照にすることで、
      // R_u が肩代わりを罰し続けるのを防ぐ。
      float u_ref_shift[nmpc::mpc::NU] = {0.f, 0.f, 0.f, 0.f, 0.f, 0.f};
      {
        int n_healthy = 0;
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          if (Motor_eta[j] > 0.5f) n_healthy++;
        if (n_healthy > 0 && n_healthy < nmpc::mpc::NU)
        {
          const int n_lost = nmpc::mpc::NU - n_healthy;
          const float up = u_hover * (float)n_lost / (float)n_healthy;
          for (int j = 0; j < nmpc::mpc::NU; ++j)
            u_ref_shift[j] = (Motor_eta[j] > 0.5f) ? up : -u_hover;
        }
      }

      // ---- 2026-09-11: 姿勢トリム（積分作用）----
      // 誤差は「遅れ補償前の実測値」を使う（積分は遅いので予測は不要、かつ予測の誤りを溜めない）。
      {
        const float e_phi   = Phi_ref   - (Phi   - Phi_bias);
        const float e_theta = Theta_ref - (Theta - Theta_bias);
        const float e_r     = Psi_ref   - (Wr    - Rbias);
        // 直前周期の出力 duty が box に張り付いていたか（アンチワインドアップ用）
        const float dmax = nmpc::mpc::U_duty_max;
        const float d6p[nmpc::mpc::NU] = {FR_duty, FL_duty, MR_duty, ML_duty, RR_duty, RL_duty};
        bool sat = false;
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          if (d6p[j] >= dmax - 1e-4f || d6p[j] <= 1e-4f) sat = true;
        att_trim_update(e_phi, e_theta, e_r, sat);
        // 積分結果は「内部の目標値をずらす」形で効かせる。
        att_trim_apply_ref(x_ref(0), x_ref(1), x_ref(4));
      }

      for (int k = 0; k < nmpc::mpc::N; ++k)
      {
        for (int i = 0; i < nmpc::mpc::NX; ++i)
          X_ref_stack(k * nmpc::mpc::NX + i) = x_ref(i);
        // 加算合成: NMPC は Δu を最適化。健全時は参照 0、故障時は上記シフト。
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          U_ref_stack(k * nmpc::mpc::NU + j) = u_ref_shift[j];
      }

      nmpc::InputVec Delta_opt;
      uint32_t t0 = time_us_32();
      nmpc::nmpc_step(g_nmpc_ctx, x0, X_ref_stack, U_ref_stack, u_hover, Delta_opt, g_nmpc_cost);
      g_nmpc_iter_us = time_us_32() - t0;

      // 加算合成 → 絶対 duty (実機モータ順序 [FR,FL,MR,ML,RR,RL])
      const float duty_max = nmpc::mpc::U_duty_max;
      auto mix_duty = [u_hover, duty_max](float du) {
        float d = u_hover + du;
        if (d < 0.0f) d = 0.0f;
        if (d > duty_max) d = duty_max;
        return d;
      };
      FR_duty = mix_duty(Delta_opt(0));
      FL_duty = mix_duty(Delta_opt(1));
      MR_duty = mix_duty(Delta_opt(2));
      ML_duty = mix_duty(Delta_opt(3));
      RR_duty = mix_duty(Delta_opt(4));
      RL_duty = mix_duty(Delta_opt(5));

      // 故障モータは 0 に落とす。box 制約でも 0 になるが、ここでも落として二重に保証する。
      {
        float *d6[nmpc::mpc::NU] = {&FR_duty, &FL_duty, &MR_duty, &ML_duty, &RR_duty, &RL_duty};
        for (int j = 0; j < nmpc::mpc::NU; ++j)
          if (Motor_eta[j] <= 0.5f) *d6[j] = 0.0f;
      }
      duty_hist_push();   // 遅れ補償用: 今周期に出す duty を履歴へ

      // ログ用 (Pref/Qref/Rref は NMPC では参照値のみ)
      Pref = Phi_ref;
      Qref = Theta_ref;
      Rref = Psi_ref;
    }

#if NMPC_BENCH_LOG
    {
      static uint32_t bench_seq = 0;
      NmpcBenchSnapshot snap = {};
      snap.seq = ++bench_seq;
      snap.arm_flag = Arm_flag;
      snap.nmpc_active = nmpc_run ? 1u : 0u;
      snap.nmpc_reset = nmpc_run ? (g_nmpc_ctx.last_was_reset ? 1u : 0u) : 0u;
      snap.reset_cause = nmpc_run ? g_nmpc_ctx.diag.last_reset_cause : 0u;
      snap.step_path = nmpc_run ? g_nmpc_ctx.diag.last_path : 0u;
      snap.tr1_applied = nmpc_run ? g_nmpc_ctx.diag.last_tr1_applied : 0u;
      snap.cycles_since_reset = nmpc_run ? g_nmpc_ctx.diag.cycles_since_reset : 0;
      snap.cnt_tr1_path = g_nmpc_ctx.diag.cnt_tr1_path;
      snap.cnt_tr1_applied = g_nmpc_ctx.diag.cnt_tr1_applied;
      snap.cnt_tr1_skipped = g_nmpc_ctx.diag.cnt_tr1_skipped;
      snap.cnt_reset_period = g_nmpc_ctx.diag.cnt_reset_period;
      snap.cnt_reset_external = g_nmpc_ctx.diag.cnt_reset_external;
      snap.cnt_reset_sr2_bad_c = g_nmpc_ctx.diag.cnt_reset_sr2_bad_c;
      snap.cnt_reset_sr2_unhealthy = g_nmpc_ctx.diag.cnt_reset_sr2_unhealthy;
      snap.cnt_reset_after_sr2 = g_nmpc_ctx.diag.cnt_reset_after_sr2;
      snap.phi = Phi - Phi_bias;
      snap.theta = Theta - Theta_bias;
      snap.p = Wp - Pbias;
      snap.q = Wq - Qbias;
      snap.r = Wr - Rbias;
      snap.phi_ref = Phi_ref;
      snap.theta_ref = Theta_ref;
      snap.psi_ref = Psi_ref;
      snap.t_ref = T_ref;
      snap.u_base = u_hover_snap;
      snap.schedule_alpha = 0.0f;
      snap.Q_phi_eff = nmpc::nmpc_cost_get_active().Q_phi;
      snap.u_fr = FR_duty;
      snap.u_fl = FL_duty;
      snap.u_mr = MR_duty;
      snap.u_ml = ML_duty;
      snap.u_rr = RR_duty;
      snap.u_rl = RL_duty;
      snap.iter_us = (float)g_nmpc_iter_us;
      snap.loop_us = (float)(time_us_32() - S_time2);
      snap.cost = g_nmpc_cost;
      snap.ftc = Ftc_mode;
      snap.failsafe = Failsafe_flag ? Failsafe_cause : FS_NONE;
      snap.rx_flags = Rx_sbus_flags;
      snap.rx_fl_run = Rx_framelost_run;
      snap.rx_fl_cnt = Rx_framelost_count;
      snap.flight_mode = Flight_mode;
      snap.lock_mode   = LockMode;
      snap.gate        = g_arm_gate_ok;
      snap.ch_ftc      = Chdata[FTC];
      snap.trim_roll   = g_trim_roll;
      snap.trim_pitch  = g_trim_pitch;
      snap.trim_yaw    = g_trim_yaw;
      nmpc_bench_update(snap);
    }
#endif
#endif

    // Logging  100Hzで情報を記憶
    logging();

    E_time2 = time_us_32();
    D_time2 = E_time2 - S_time2;
  }
}

// --------------------------------Rocking wings---------------------------------------
float rocking_wings(float stick)
{
  float angle = 20; //[deg]
  float f = 5.0;    //[Hz]

  if (Rocking_timer < 2.0)
  {
    Rocking_timer = Rocking_timer + 0.01;
    // rgbled_rocking();
    return angle * M_PI / 180 * sin(f * 2 * M_PI * Rocking_timer);
  }
  else
  {
    rgbled_normal();
    return stick;
  }
}

void logging(void)
{
  // LOG スイッチが入っている間、100Hz でリングバッファに書き続ける。
  // 満杯になっても止めず、最も古い行から上書きする（＝常に直近 21.8 秒が残る）。
  if (Chdata[LOG] > 200)
  {
    if (Logflag == 0)
    {
      Logflag = 1;
      LogRow = 0;
      Log_wrapped = 0;
    }

    float *L = &Logdata[(uint32_t)LogRow * DATANUM];
    uint8_t k = 0;
    L[k++] = Phi - Phi_bias;                 //  1 phi   [rad]
    L[k++] = Theta - Theta_bias;             //  2 theta
    L[k++] = Wp - Pbias;                     //  3 p     [rad/s]
    L[k++] = Wq - Qbias;                     //  4 q
    L[k++] = Wr - Rbias;                     //  5 r
    L[k++] = Phi_ref;                        //  6 phi_ref
    L[k++] = Theta_ref;                      //  7 theta_ref
    L[k++] = Psi_ref;                        //  8 psi_ref (ヨー率指令)
    L[k++] = T_ref;                          //  9 T_ref
    L[k++] = T_ref * T_ref_coe;              // 10 u_base
    L[k++] = FR_duty;                        // 11 u_fr
    L[k++] = FL_duty;                        // 12 u_fl
    L[k++] = MR_duty;                        // 13 u_mr
    L[k++] = ML_duty;                        // 14 u_ml
    L[k++] = RR_duty;                        // 15 u_rr
    L[k++] = RL_duty;                        // 16 u_rl
#ifdef USE_NMPC
    L[k++] = (float)g_nmpc_iter_us;                        // 17 iter_us
    L[k++] = (float)(time_us_32() - S_time2);              // 18 loop_us (今周期の経過)
    L[k++] = (float)g_nmpc_ctx.diag.cycles_since_reset;    // 19 cycles
#else
    L[k++] = 0.0f;
    L[k++] = (float)(time_us_32() - S_time2);
    L[k++] = 0.0f;
#endif
    L[k++] = (float)(Failsafe_flag ? Failsafe_cause : FS_NONE);  // 20 failsafe
    L[k++] = (float)Ftc_mode;                                    // 21 ftc
    L[k++] = sqrtf(Ax * Ax + Ay * Ay + Az * Az);                 // 22 acc_norm [m/s^2]
    // ---- 2026-09-10 追加: 推定器とソルバの内部状態 ----
    L[k++] = Xe(4, 0) - Pbias;                                   // 23 ekf_bp  EKF バイアス状態 - 静止平均 [rad/s]
    L[k++] = Xe(5, 0) - Qbias;                                   // 24 ekf_bq
    L[k++] = Xe(6, 0) - Rbias;                                   // 25 ekf_br
    L[k++] = Ekf_drift_phi;                                      // 26 ekf_dphi   姿勢角変化率 - ジャイロ由来 [rad/s]
    L[k++] = Ekf_drift_theta;                                    // 27 ekf_dtheta
#ifdef USE_NMPC
    {
      // ソルバ内部: 双対ノルム、線形化点ノルム、経路+box 有効数
      //   solver_code = path*10 + box_n
      //   path: 0=通常(TR1/SR2) 1=フル再構築 2=TR1 更新を捨てた周期
      int box_n = 0;
      for (int j = 0; j < nmpc::mpc::NU; ++j)
      {
        const float z = g_nmpc_ctx.ws.z(j);
        if (z <= g_nmpc_ctx.scratch.du_min(j) + 1e-4f || z >= g_nmpc_ctx.scratch.du_max(j) - 1e-4f) box_n++;
      }
      int path = 0;
      if (g_nmpc_ctx.last_was_reset) path = 1;
      else if (!g_nmpc_ctx.diag.last_tr1_applied) path = 2;
      L[k++] = g_nmpc_ctx.ws.lambda.norm();                      // 28 lam_norm
      L[k++] = g_nmpc_ctx.scratch.U_lin.norm();                  // 29 ulin_norm
      L[k++] = (float)(path * 10 + box_n);                       // 30 solver_code
    }
#else
    L[k++] = 0.0f; L[k++] = 0.0f; L[k++] = 0.0f;
#endif

    if (++LogRow >= LOGROWS)
    {
      LogRow = 0;
      Log_wrapped = 1;
    }
  }
  else
  {
    // スイッチを戻しても中身は消さない（墜落後に吸い出せるようにするため）。
    // 次に ON にしたときだけ先頭から取り直す。
    if (Logflag > 0)
      Logflag = 0;
  }
}

void log_output(void)
{
  static uint32_t out_i = 0;
  const uint32_t count = Log_wrapped ? (uint32_t)LOGROWS : (uint32_t)LogRow;

  if (out_i == 0)
  {
    printf("#nmpc_log 100Hz rows=%lu cols=%u (%.1f s)\r\n",
           (unsigned long)count, (unsigned)DATANUM, count * 0.01f);
#ifdef USE_NMPC
    // nmpc_params.hpp の各 namespace をそのまま印字（後解析でソースと突合できるように）
    printf("#params: N=%d dt=%.3f T_max=%.3f mass=%.3f Ct=%.3e CQ=%.3e C_QT=%.4f\r\n",
           nmpc::mpc::N, nmpc::mpc::dt, nmpc::physical::T_max, nmpc::physical::mass,
           nmpc::physical::Ct, nmpc::physical::CQ, nmpc::physical::C_QT);
    printf("#physical: Ix=%.4e Iy=%.4e Iz=%.4e Cp=%.4f Cq=%.4f Cr=%.4f\r\n",
           nmpc::physical::Ix, nmpc::physical::Iy, nmpc::physical::Iz,
           nmpc::physical::Cp, nmpc::physical::Cq, nmpc::physical::Cr);
    printf("#mpc: NX=%d NU=%d U_duty_max=%.2f damping_eps=%.1e\r\n",
           nmpc::mpc::NX, nmpc::mpc::NU,
           nmpc::mpc::U_duty_max, nmpc::mpc::damping_eps);
    if (Bench_mode != BENCH_OFF)
      printf("#bench: mode=%u u0=%.3f delta=%.3f sign=%+.0f "
             "t_spin=%.1f t_step=%.1f t_decay=%.1f\r\n",
             (unsigned)Bench_mode, Bench_u0, Bench_delta, Bench_sign,
             Bench_t_spin, Bench_t_step, Bench_t_decay);
    {
      const nmpc::NmpcCostParams cp = nmpc::nmpc_cost_get_active();
      printf("#solver: lambda_reset=%lu nonfinite=%lu ulin_reset=%lu ekf_reset=%lu ekf_bias_reset=%lu admm_rho=%.0f admm_iter=%d\r\n",
           (unsigned long)g_nmpc_ctx.diag.cnt_admm_lambda_reset,
           (unsigned long)g_nmpc_ctx.diag.cnt_nonfinite,
           (unsigned long)g_nmpc_ctx.diag.cnt_ulin_reset,
           (unsigned long)Ekf_reset_count,
           (unsigned long)Ekf_bias_reset_count,
           (double)nmpc::rti::admm_rho, nmpc::rti::admm_max_iter);
    printf("#cost: Q_phi=%.1f Q_theta=%.1f Q_p=%.1f Q_q=%.1f Q_r=%.1f R_u=%.1f term=%.1f\r\n",
             cp.Q_phi, cp.Q_theta, cp.Q_p, cp.Q_q, cp.Q_r, cp.R_u, cp.terminal_scale);
    }
    printf("#rti: reset_period=%d admm_max_iter=%d admm_rho=%.1f rebuild_max_u_hover=%.2f\r\n",
           nmpc::rti::reset_period, nmpc::rti::admm_max_iter, nmpc::rti::admm_rho,
           nmpc::rti::rebuild_max_u_hover);
    printf("#delay_comp: steps=%d pred_nonfinite=%lu legacy_solver=%d\r\n",
           nmpc::rti::delay_comp_steps, (unsigned long)g_delay_pred_nonfinite, (int)NMPC_LEGACY_SOLVER);
#endif
    printf("#failsafe: 0=none 1=rx_lost 2=rx_timeout 3=attitude 4=rate 5=overg\r\n");
    printf("#arm_gate: reinit=%lu wait_ms=%lu limit_dps=1.0 hold_s=1.0 warm_s=1.5\r\n",
           (unsigned long)g_arm_reinit_count, (unsigned long)g_arm_gate_wait_ms);
    printf("#ekf: bias_reset=%lu drift_lim_dps=%.1f lpf_tau_s=0.3 run_n=%u\r\n",
           (unsigned long)Ekf_bias_reset_count, (double)(EKF_DRIFT_LIMIT * 180.0f / (float)M_PI),
           (unsigned)EKF_DRIFT_RUN_N);
    printf("#solver_code: path*10+box_n  path 0=normal 1=rebuild 2=tr1_skip\r\n");
    printf("#cols: t,phi,theta,p,q,r,phi_ref,theta_ref,psi_ref,T_ref,u_base,"
           "u_fr,u_fl,u_mr,u_ml,u_rr,u_rl,iter_us,loop_us,cycles,failsafe,ftc,acc_norm,"
           "ekf_bp,ekf_bq,ekf_br,ekf_dphi,ekf_dtheta,lam_norm,ulin_norm,solver_code\r\n");
    if (count == 0)
    {
      printf("#empty (LOG スイッチで記録していません)\r\n");
    }
  }

  if (out_i < count)
  {
    // 最も古い行から出す。一周していれば書き込み位置が最古。
    const uint32_t row = Log_wrapped ? (((uint32_t)LogRow + out_i) % LOGROWS) : out_i;
    const float *L = &Logdata[row * DATANUM];
    printf("%.2f", out_i * 0.01f);
    for (uint8_t i = 0; i < DATANUM; i++)
      printf(",%.5f", L[i]);
    printf("\r\n");
    out_i++;
  }
  else
  {
    printf("#end\r\n");
    out_i = 0;
    Arm_flag = 3;
    Logoutputflag = 0;
    LockMode = 0;
    Log_time = 0.0;
    LogdataCounter = 0;
  }
}

void gyroCalibration(void)
{
  float wp, wq, wr;
  float sump, sumq, sumr;
  uint16_t N = 400;
  for (uint16_t i = 0; i < N; i++)
  {
    sensor_read();
    sump = sump + Wp;
    sumq = sumq + Wq;
    sumr = sumr + Wr;
  }
  Pbias = sump / N;
  Qbias = sumq / N;
  Rbias = sumr / N;
}

void debug_print_gyro(void)
{
  printf("Arm=%d Lock=%d Wp=%.4f Wq=%.4f Wr=%.4f mdps:%.1f %.1f %.1f\r\n",
         Arm_flag, LockMode, Wp, Wq, Wr,
         angular_rate_mdps[0], angular_rate_mdps[1], angular_rate_mdps[2]);
}

// OpenMV通信用----------------------------------------------------------------------------------------------------------------------------
//  void processReceiveData(){
//
//    //   //姿勢によるライン検知の誤差補正
//    //   x_diff = -x_diff;
//    //   angle_diff = -angle_diff*M_PI/180.0;
//    //   x_alpha = atan2(x_diff,118);
//    //   x_diff_dash = 700*tan(Phi + x_alpha); //角度補正

//     //座標変換----------------------------------------------------------------------------------------------------------------------------------------------------------------------
//     // float e11,e12,e13,e21,e22,e23,e31,e32,e33;//透視変換の内部パラメータ
//     // float E11,E12,E13,E21,E22,E23,E31,E32,E33;//方向余弦行列
//     // float fx,fy,cx,cy,u1_camera,v1_camera,u2_camera,v2_camera,u1_camera_dash,u2_camera_dash,v1_camera_dash,v2_camera_dash,x1_camera,x2_camera,y1_camera,y2_camera,z1_camera,z2_camera;//透視変換
//     // float x1_drone,x2_drone,y1_drone,y2_drone,z1_drone,z2_drone;//カメラ座標から機体座標へ座標変換
//     // float X1_inertia,Y1_inertia,Z1_inertia,X2_inertia,Y2_inertia,Z2_inertia,X0_1_inertia,Y0_1_inertia,Z0_1_inertia,X0_2_inertia,Y0_2_inertia,Z0_2_inertia;//機体座標から慣性座標に座標変換

//     // //画像カメラからカメラ座標への座標変換
//     // u1_camera = x_1_dash;
//     // v1_camera = y_1_dash;
//     // u2_camera = x_2_dash;
//     // v2_camera = y_2_dash;

//     // //焦点距離は2.8mm
//     // //受光素子数が横：120,縦：120
//     // //ピクセルサイズ：2.8㎛×2.8㎛

//     // //画像座標、カメラ座標の原点を中心に移動
//     // u1_camera_dash = 0.0028*u1_camera + 0.0014 - (0.0028*160)/2;
//     // u2_camera_dash = 0.0028*u2_camera + 0.0014 - (0.0028*160)/2;
//     // v1_camera_dash = 0.0028*v1_camera + 0.0014 - (0.0028*120)/2;
//     // v2_camera_dash = 0.0028*v2_camera + 0.0014 - (0.0028*120)/2;

//     // //イメージセンサの長さ=ピクセルサイズ*受光素子数
//     // fx = (160/0.336)*2.8;//(横の受光素子数/イメージセンサの横の長さ)*mm単位の焦点距離
//     // fy = (120/0.448)*2.8;//(縦の受光素子数/イメージセンサの縦の長さ)*mm単位の焦点距離

//     // cx = 80.5*0.0028;//x軸方向の光学中心、（横のピクセル数*ピクセルサイズ）/2
//     // cy = 60.5*0.0028;//y軸方向の光学中心、（縦のピクセル数*ピクセルサイズ）/2

//     // //透視変換の内部パラメータ（画像座標→カメラ座標）
//     // e11 = 1/fx;
//     // e12 = 0;
//     // e13 = -cx/fy;
//     // e21 = 0;
//     // e22 = 1/fy;
//     // e23 = -cy/fy;
//     // e31 = 0;
//     // e32 = 0;
//     // e33 = 1;

//     // //透視変換(画像座標→カメラ座標)
//     // x1_camera = Kalman_alt*e11*u1_camera_dash;
//     // y1_camera = Kalman_alt*e22*v1_camera_dash;
//     // x2_camera = Kalman_alt*e11*u2_camera_dash;
//     // y2_camera = Kalman_alt*e22*v2_camera_dash;
//     // z1_camera = Kalman_alt*(e31*u1_camera + e32*v1_camera + 1);
//     // z2_camera = Kalman_alt*(e31*u2_camera + e32*v2_camera + 1);

//     // //カメラ座標から機体座標への座標変換
//     // y1_drone = x1_camera;
//     // x1_drone = -y1_camera;
//     // y2_drone = x2_camera;
//     // x2_drone = -y2_camera;
//     // z1_drone = z1_camera;
//     // z2_drone = z2_camera;

//     // //クォータニオンの計算
//     // q0 = Xe(0,0);
//     // q1 = Xe(1,0);
//     // q2 = Xe(2,0);
//     // q3 = Xe(3,0);

//     // //転置後の方向余弦行列（機体座標→慣性座標）
//     // E11 = q0^2 + q1^2 + q2^2 + q3^2;
//     // E12 = 2*(q1*q2 - q0*q3);
//     // E13 = 2*(q1*q3 - q0*q3);
//     // E21 = 2*(q1*q2 - q0*q3);
//     // E22 = q0^2 - q1^2 + q2^2 - q3^2;
//     // E23 = 2*(q1*q3 - q0*q1);
//     // E31 = 2*(q1*q3 - q0*q2);
//     // E32 = 2*(q2*q3 + q0*q1);
//     // E33 = q0^2 - q1^2 - q2^2 + q3^2;

//     // //機体座標から慣性座標への座標変換
//     // X1_inertia = E11*x1_drone + E12*y1_drone + E13*z1_drone;
//     // Y1_inertia = E21*x1_drone + E22*y1_drone + E23*z1_drone;
//     // Z1_inertia = E31*x1_drone + E32*y1_drone + E33*z1_drone;

//     // X2_inertia = E11*x2_drone + E12*y2_drone + E13*z2_drone;
//     // Y2_inertia = E21*x2_drone + E22*y2_drone + E23*z2_drone;
//     // Z2_inertia = E31*x2_drone + E32*y2_drone + E33*z2_drone;

//     // //カメラから得られた2点の画像座標を慣性座標上のドローンの位置の座標
//     // X0_1_inertia = 0 - X1_inertia;
//     // Y0_1_inertia = 0 - Y1_inertia;
//     // Z0_1_inertia = 0 - Z1_inertia;

//     // X0_2_inertia = 0 - X2_inertia;
//     // Y0_2_inertia = 0 - Y2_inertia;
//     // Z0_2_inertia = 0 - Z2_inertia;

//     // //慣性空間に変換したドローン自身の2点の座標から直線の方程式を出す
//     // float Inclination,b;

//     // Inclination = (Y0_2_inertia - Y0_1_inertia)/(X0_2_inertia - X0_1_inertia);//直線の方程式の傾き
//     // b = ((X0_2_inertia*Y0_1_inertia)/X0_1_inertia - Y0_2_inertia)/(X0_2_inertia/X0_1_inertia - 1);//直線の方程式の切片

//     // Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
//     // Line_velocity = Velocity_filter.update(Xn_est_1); //速度
//     // Line_range = Range_filter.update(Xn_est_2); //横ずれ
//     // Psi = Angle_filter.update(Xn_est_3); //ラインとの角度

//     // // current_time = time_us_64();
//     // //printf("x : %9.6f\n",x_diff);
//     // printf("red_circle : %f\n",red_circle);
//     // printf("length : %f\n",length);
// }
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

// }

//OpenMV通信用
void processReceiveData(){

  char* clear_data = buffer;
  clear_data++;//(をスキップ
  clear_data[strlen(clear_data) -1 ] = '\0';//)をヌル文字に置き換え
  char* token;

  // if (Flight_mode == HOVERING)
  // {
  //   token = strtok(clear_data,",");
  //   if (token != NULL){
  //     TOL_x_diff = atof(token);
  //   }
  //   token = strtok(NULL,",");
  //   if (token != NULL){
  //     TOL_y_diff = atof(token);
  //   }
  //   // printf("%9.6f \n",TOL_x_diff);
  //   // printf("%9.6f \n",TOL_y_diff);
  // }

  // if (Flight_mode == LINETRACE)
  if (Flight_mode == 100)
  {
     token = strtok(clear_data,",");
      if (token != NULL){
        x_diff = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        angle_diff = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        gap_number = atof(token);
      }
      token = strtok(NULL,",");
      if (token != NULL){
        line_number = atof(token);
      }
      //x_diff_dash = ((2 * 500 * tan(35) * -x_diff) / 160) - (500 * tan(Phi));
      // x_diff_dash = ((x_diff * 497.46) / 80) - (700*tan(Phi));
      x_alpha = atan2(x_diff,118);
      x_diff_dash = 700 * tan(Phi - x_alpha);
      Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
      Line_range = Xn_est_2; //横ずれ
      Line_velocity = Xn_est_1; //速度
      // current_time = time_us_64();
      // printf("x_diff : %9.6f angle_diff : %9.6f gap : %9.6f  line_num : %9.6f\n",x_diff,angle_diff,gap_number,line_number);


    // token = strtok(clear_data,",");
    // if (token != NULL){
    //   TOL_x_diff = atof(token);
    // }
    // token = strtok(NULL,",");
    // if (token != NULL){
    //   TOL_y_diff = atof(token);
    // }
    // printf("%9.6f \n",TOL_x_diff);
    // printf("%9.6f \n",TOL_y_diff);

    // if(landing_counter ==1){
    //   token = strtok(clear_data,",");
    //   if (token != NULL){
    //     TOL_x_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     TOL_y_diff = atof(token);
    //   }
    //   // TOL_x_alpha = atan2(TOL_x_diff,118);
    //   // TOL_y_alpha = atan2(TOL_y_diff,87);
    //   // TOL_x_diff_dash = 700 * tan(Phi - TOL_x_alpha);
    //   // TOL_y_diff_dash = 700 * tan(Theta - TOL_y_alpha);
    //   // printf("%9.6f %9.6f\n",TOL_x_diff,TOL_x_diff_dash);
    //   // printf("%9.6f %9.6f\n",TOL_y_diff,TOL_y_diff_dash);
    // }
    // else{
    //   token = strtok(clear_data,",");
    //   if (token != NULL){
    //     x_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     angle_diff = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     gap_number = atof(token);
    //   }
    //   token = strtok(NULL,",");
    //   if (token != NULL){
    //     line_number = atof(token);
    //   }
    //   //x_diff_dash = ((2 * 500 * tan(35) * -x_diff) / 160) - (500 * tan(Phi));
    //   // x_diff_dash = ((x_diff * 497.46) / 80) - (700*tan(Phi));
    //   x_alpha = atan2(x_diff,118);
    //   x_diff_dash = 700 * tan(Phi - x_alpha);
    //   Kalman_holizontal(x_diff_dash,angle_diff,(Wp - Pbias),(Wr - Rbias),(Phi - Phi_bias));
    //   Line_range = Xn_est_2; //横ずれ
    //   Line_velocity = Xn_est_1; //速度
    //   // current_time = time_us_64();
    //   printf("x_diff : %9.6f angle_diff : %9.6f gap : %9.6f  line_num : %9.6f\n",x_diff,angle_diff,gap_number,line_number);
    //   //printf("x : %9.6f Est : %9.6f dash : %9.6f\n",x_diff,Line_range,x_diff_dash);
    //   // printf("angle : %9.6f est : %9.6f\n",angle_diff,Xn_est_3);
    //   //printf("%9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f %9.6f\n",(current_time - start_time )/1000000,x_diff,Line_range,x_diff_dash,Theta,Phi,angle_diff,Xn_est_3,Xn_est_1,x_alpha);
    // }
  }

  // 旧 REDCIRCLE モードは削除 (red_circle の取り込みも停止)

  // printf("x : %9.6f\n",x_diff);
  // printf("angle : %9.6f\n",angle_diff);
  // printf("est velocity: %9.6f\n",Xn_est_1);
  // printf("y : %9.6f, est : %9.6f\n",x_diff,Xn_est_2);
  // printf("psi : %9.6f, est : %9.6f\n",angle_diff,Xn_est_3);
}

void receiveData(char c){
  if (buffer_index < BUFFER_SIZE - 1){
    buffer[buffer_index++] = c;
  }
  //終了条件のチェック
  // if (c == '\n'){
  if (c == ')'){
    //buffer[buffer_index] = '\0'; //文字列の終端にヌル文字を追加
    processReceiveData();
    buffer_index = 0; //バッファをリセット
  }
}

void sensor_read(void)
{
  float mx1, my1, mz1, mag_norm, acc_norm, rate_norm;

  imu_mag_data_read();
  Ax = -acceleration_mg[0] * GRAV * 0.001;
  Ay = -acceleration_mg[1] * GRAV * 0.001;
  Az = acceleration_mg[2] * GRAV * 0.001;
  Wp = angular_rate_mdps[0] * M_PI * 5.55555555e-6; // 5.5.....e-6=1/180/1000
  Wq = angular_rate_mdps[1] * M_PI * 5.55555555e-6;
  Wr = -angular_rate_mdps[2] * M_PI * 5.55555555e-6;
  Mx0 = -magnetic_field_mgauss[0];
  My0 = magnetic_field_mgauss[1];
  Mz0 = -magnetic_field_mgauss[2];

  // 加速度・角速度のリミッター
  acc_norm = sqrt(Ax * Ax + Ay * Ay + Az * Az);
  // 旧値 800 [m/s^2] (=81.5g) は LSM9DS1 ±16g 設定 (最大 157 m/s^2, 3軸合成 272) では
  // 到達不能で、OverG は一度も発火していなかった。誤検知で飛行中に止まる方が危険なので、
  // まずは「明らかに衝突」の水準に置く。係留ログで分布を見てから下げること。
  if (acc_norm > FS_ACC_LIMIT)
    OverG_flag = 1;
  // Acc_norm = acc_filter.update(acc_norm);
  // rate_norm = sqrt(Wp*Wp + Wq*Wq + Wr*Wr);
  // if (rate_norm > 70.0) OverG_flag =1;

  /*地磁気校正データ
  回転行列
  [[ 0.65330968  0.75327755 -0.07589064]
  [-0.75666134  0.65302622 -0.03194321]
  [ 0.02549647  0.07829232  0.99660436]]
  中心座標
  122.37559195017053 149.0184454603531 -138.99116060635413
  W
  -2.432054387460946
  拡大係数
  0.003077277151877191 0.0031893151610213463 0.0033832794976645804

  //回転行列
  const float rot[9]={0.65330968, 0.75327755, -0.07589064,
                    -0.75666134, 0.65302622, -0.03194321,
                      0.02549647, 0.07829232,  0.99660436};
  //中心座標
  const float center[3]={122.37559195017053, 149.0184454603531, -138.99116060635413};
  //拡大係数
  const float zoom[3]={0.003077277151877191, 0.0031893151610213463, 0.0033832794976645804};
  */
  // 回転行列
  const float rot[9] = {-0.78435472, -0.62015392, -0.01402787,
                        0.61753358, -0.78277935, 0.07686857,
                        -0.05865107, 0.05162955, 0.99694255};
  // 中心座標
  const float center[3] = {-109.32529343620176, 72.76584808916506, 759.2285249891385};
  // 拡大係数
  const float zoom[3] = {0.002034773458122364, 0.002173892202021849, 0.0021819494099235273};

  // 回転・平行移動・拡大
  mx1 = zoom[0] * (rot[0] * Mx0 + rot[1] * My0 + rot[2] * Mz0 - center[0]);
  my1 = zoom[1] * (rot[3] * Mx0 + rot[4] * My0 + rot[5] * Mz0 - center[1]);
  mz1 = zoom[2] * (rot[6] * Mx0 + rot[7] * My0 + rot[8] * Mz0 - center[2]);
  // 逆回転
  Mx = rot[0] * mx1 + rot[3] * my1 + rot[6] * mz1;
  My = rot[1] * mx1 + rot[4] * my1 + rot[7] * mz1;
  Mz = rot[2] * mx1 + rot[5] * my1 + rot[8] * mz1;

  mag_norm = sqrt(Mx * Mx + My * My + Mz * Mz);
  Mx /= mag_norm;
  My /= mag_norm;
  Mz /= mag_norm;

  // // 高度センサーから値受け取るコード
  // altitude_count = altitude_count + 1;
  // if (altitude_count == 8){
  //   altitude_count = 0;
  //   uint8_t checkdata[2];
  //   int result = i2c_read_blocking(I2C_PORT, dev, checkdata, sizeof(checkdata), false);
  //   if (result != 2)
  //   {
  //     // I2C通信エラーチェック
  //     // エラーが発生した場合、I2C通信が切断されたとみなす
  //     // ここで適切なエラーハンドリングを行う
  //     Flight_mode = NORMAL;
  //     i2c_connect = 0;
  //     printf("I2C通信エラーが発生しました。 %4d\n", result);
  //   }
  //   else
  //   {
  //     // printf("I2C通信接続できました。 %4d\n",result);
  //     // 高度センサーから値受け取るコード
  //     if (isDataReady == 0)
  //     {
  //       Status = VL53L1X_CheckForDataReady(dev, &isDataReady);
  //     }
  //     else if (isDataReady == 1)
  //     {
  //       isDataReady = 0;
  //       Status = VL53L1X_GetRangeStatus(dev, &rangeStatus);
  //       Status = VL53L1X_GetDistance(dev, &distance);
  //       Status = VL53L1X_ClearInterrupt(dev);
  //       // z_acc  = Az-9.80665;
  //       z_acc = Az - 9.76548;
  //       lotate_altitude_init(Theta, Psi, Phi);
  //       lotated_distance = lotate_altitude(distance);
  //       Kalman_alt = Kalman_PID(lotated_distance, z_acc);
  //       altitude = mu_Yn_est(1,0);
  //       // if ((Kalman_alt - last_Kalman_alt) > 500 || (Kalman_alt - last_Kalman_alt) < 500)
  //       // {
  //       //   Kalman_alt = last_Kalman_alt;
  //       // }
  //       // else{
  //       //   last_Kalman_alt = Kalman_alt;
  //       // }
  //       // printf("%f senser_dis:\r\n",lotated_distance);
  //       // printf("%f z_acc:\r\n",z_acc);
  //       // printf("%f kalman_v:\r\n",mu_Yn_est(0,0));
  //       // if(Flight_mode !=HOVERING && Flight_mode!=REDCIRCLE){
  //       // printf("%f mu:\r\n",mu_Yn_est(1,0));
  //       // printf("%f altitude:\r\n",altitude);
  //       // printf("%f Kalman_alt:\r\n",Kalman_alt);
  //       // }
  //     }
  //   }
  // }

  // //OpenMV通信用
  // if (Flight_mode == LINETRACE && i2c_connect == 1){
  // if (Flight_mode == LINETRACE){

  //   //着陸のFlightModeを決める
  //   if(landing_counter == 1 ){
  //   uart_putc(UART_ID2,'3');
  //   }
  //   else{
  //     uart_putc(UART_ID2,'1');
  //   }

  //   while (uart_is_readable(UART_ID2)){
  //     char c = uart_getc(UART_ID2);
  //     receiveData(c);
  //   }
  // }


  // if(Flight_mode == HOVERING)
  // {
  //   uart_putc(UART_ID2,'3');
  //   if (uart_is_readable(UART_ID2))
  //   {
  //     char c = uart_getc(UART_ID2);
  //     receiveData(c);
  //   }
  // }

  // // OpenMV通信用 データ受信 カメラの中心と対象物との距離を測定
  // // if ((Flight_mode == REDCIRCLE) && (i2c_connect == 1))
  // if ((Flight_mode == REDCIRCLE))
  // {
  //   uart_putc(UART_ID2,'2');
  //   printf("REDCIRCLE\r\n");
  //   if (uart_is_readable(UART_ID2))
  //   {
  //     // float *a;
  //     // // extern float length;
  //     // int i;
  //     // unsigned char buf[BUFFER_SIZE];
  //     // for (i = 0; i < BUFFER_SIZE; i++)
  //     // {
  //     //   buf[i] = uart_getc(UART_ID2);
  //     // }
  //     // a = (float *)buf;
  //     // length = *a;
  //     char c = uart_getc(UART_ID2);
  //     receiveData(c);

  //     // loop_count+=1;

  //     // printf("loop_count:%d\n" , loop_count);

  //     // if (loop_count > 400){
  //     //   loop_count = 0;
  //     //   red_count = 0;
  //     // }    

  //     // if (red_circle == 1){
  //     //   red_count+=1;
  //     // }

  //     // printf("red_count:%d\n" , red_count);

  //     // if (red_count > 20 && loop_count <=400){
  //     //   release_flag = 1;
  //     //   // payload_relese();
  //     //   // printf("release\n");
  //     // }

  //     // if (release_flag == 1){
  //     //   payload_relese();
  //     //   red_count = 0;
  //     //   loop_count = 0;
  //     // }

  //     // else{
  //     //   payload_hook();
  //     // }

  //     // printf("%f %f\n",Theta,length);
  //     // 自動物資投下
  //     // printf("Theta:%f\n", Theta);
  //     // printf("Phi:%f\n", Phi);
  //     // printf("length:%f\n", length);

  //     // float degreePhi = Phi * (180 / pi);
  //     // float degreeTheta = Theta * (180 / pi);

  //     // float corrected_length_Phi = length * fabsf(cos(degreePhi));
  //     // float corrected_length_Theta = length * fabsf(cos(degreeTheta));

  //     // float corrected_length = (corrected_length_Phi + corrected_length_Theta) / 2;

  //     // printf("%f\n", corrected_length);

  //     // if (corrected_length <= 10)
  //     // {
  //     //   // if(length <= 10){
  //     //   length_count++;
  //     // }
  //     // else
  //     // {
  //     //   length_count = 0;
  //     // }

  //     // if (length_count > 3 && length_count <= 10)
  //     // {
  //     //   payload_relese();
  //     // }
  //     // else
  //     // {
  //     //   payload_hook();
  //     // }
  //     // printf("%.8f\n",D_time);
  //   }
  // }
}

void variable_init(void)
{
  // Variable Initalize
  Xe << 1.00, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
  Xp = Xe;

  Q << 6.0e-5, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 5.0e-5, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 2.8e-5, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 5.0e-5, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 5.0e-5, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 5.0e-5;

  R << 1.701e0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 2.799e0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 1.056e0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 2.3e-1, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.4e-1, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.49e-1;

  G << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0,
      -1.0, 1.0, -1.0, 0.0, 0.0, 0.0,
      -1.0, -1.0, 1.0, 0.0, 0.0, 0.0,
      1.0, -1.0, -1.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

  Beta << 0.0, 0.0, 0.0;

  P << 1e0, 0, 0, 0, 0, 0, 0,
      0, 1e0, 0, 0, 0, 0, 0,
      0, 0, 1e0, 0, 0, 0, 0,
      0, 0, 0, 1e0, 0, 0, 0,
      0, 0, 0, 0, 1e0, 0, 0,
      0, 0, 0, 0, 0, 1e0, 0,
      0, 0, 0, 0, 0, 0, 1e0;
}

void printPQR(void)
{
  volatile int m = 0;
  volatile int n = 0;
  // Print P
  printf("#P\n");
  for (m = 0; m < 7; m++)
  {
    printf("# ");
    for (n = 0; n < 7; n++)
    {
      printf("%12.4e ", P(m, n));
    }
    printf("\n");
  }
  // Print Q
  printf("#Q\n");
  for (m = 0; m < 6; m++)
  {
    printf("# ");
    for (n = 0; n < 6; n++)
    {
      printf("%12.4e ", Q(m, n));
    }
    printf("\n");
  }
  // Print R
  printf("#R\n");
  for (m = 0; m < 6; m++)
  {
    printf("# ");
    for (n = 0; n < 6; n++)
    {
      printf("%12.4e ", R(m, n));
    }
    printf("\n");
  }
}

void output_data(void)
{
  printf("%9.3f,"
         "%13.8f,%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%6lu,%6lu,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f,"
         "%13.8f,%13.8f,%13.8f"
         //"%13.8f"
         "\n",
         Elapsed_time // 1
         ,
         Xe(0, 0), Xe(1, 0), Xe(2, 0), Xe(3, 0) // 2~5
         ,
         Xe(4, 0), Xe(5, 0), Xe(6, 0) // 6~8
         //,Phi-Phi_bias, Theta-Theta_bias, Psi-Psi_bias//6~8
         ,
         D_time, D_time2 // 10,11
         ,
         Ax, Ay, Az // 11~13
         ,
         Wp, Wq, Wr // 14~16
         ,
         Mx, My, Mz // 17~19
         //,mag_norm
  ); // 20
}
void output_sensor_raw_data(void)
{
  printf("%9.3f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f,"
         "%13.5f,%13.5f,%13.5f"
         "\n",
         Elapsed_time // 1
         ,
         Ax, Ay, Az // 2~4
         ,
         Wp, Wq, Wr // 5~7
         ,
         Mx, My, Mz // 8~10
  );                // 20
}

void kalman_filter(void)
{
  // Kalman Filter
  float dt = 0.01;
  Omega_m << Wp, Wq, Wr;
  Z << Ax, Ay, Az, Mx, My, Mz;
  ekf(Xp, Xe, P, Z, Omega_m, Q, R, G * dt, Beta, dt);
}

// PID::PID()
// {
//   m_kp=1.0e-8;
//   m_ti=1.0e8;
//   m_td=0.0;
//   m_integral=0.0;
//   m_filter_time_constant=0.01;
//   m_filter_output=0.0;
//   m_err=0.0;
//   m_h=0.01;
// }

// void PID::set_parameter(
//     float kp,
//     float ti,
//     float td,
//     float filter_time_constant,
//     float h)
// {
//   m_kp=kp;
//   m_ti=ti;
//   m_td=td;
//   m_filter_time_constant=filter_time_constant;
//   m_h=h;
// }

// void PID::reset(void)
// {
//   m_integral=0.0;
//   m_filter_output=0.0;
//   m_err=0.0;
//   m_err2=0.0;
//   m_err3=0.0;
// }

// void PID::i_reset(void)
// {
//   m_integral=0.0;
// }
// void PID::printGain(void)
// {
//   printf("#Kp:%8.4f Ti:%8.4f Td:%8.4f Filter T:%8.4f h:%8.4f\n",m_kp,m_ti,m_td,m_filter_time_constant,m_h);
// }

// float PID::filter(float x)
// {
//   m_filter_output = m_filter_output * m_filter_time_constant/(m_filter_time_constant + m_h)
//                   + x * m_h/(m_filter_time_constant + m_h);
//   return m_filter_output;
// }

// float PID::update(float err)
// {
//   float d;
//   m_integral = m_integral + m_h * err;
//   if(m_integral> 30000.0)m_integral = 30000.0;
//   if(m_integral<-30000.0)m_integral =-30000.0;
//   m_filter_output = filter((err-m_err3)/m_h);
//   m_err3 = m_err2;
//   m_err2 = m_err;
//   m_err  = err;
//   return m_kp*(err + m_integral/m_ti + m_td * m_filter_output);
// }

// Filter::Filter()
// {
//   m_state = 0.0;
//   m_T = 0.0025;
//   m_h = 0.0025;
// }

// void Filter::reset(void)
// {
//   m_state = 0.0;
// }

// void Filter::set_parameter(float T, float h)
// {
//   m_T = T;
//   m_h = h;
// }

// float Filter::update(float u)
// {
//   m_state = m_state * m_T /(m_T + m_h) + u * m_h/(m_T + m_h);
//   m_out = m_state;
//   return m_out;
// }

void unique_mission(){
  //もし0.4sで宙返りができるなら
float P_max=20*pi ;
float P_angle=50*pi ;
uint16_t flip_s;
float q_rate;
if((Chdata[SERVO] < 200) && (Chdata[LOG] < 200) && (Chdata[LINETRACE] < 200) && (Chdata[HOVERING] < 200)){
  if(0<=flip_s && flip_s<80)
  {
    T_ref=0.5;
    // flip_sを0.0025:0.2=1:flip_sで求めた。
 
      T_ref+=0.03;
      q_rate= P_angle*flip_s;
  }  
  if(0.2<=flip_s&&flip_s<0.4)
    q_rate=-P_angle*flip_s+10* pi ;
  flip_s ++;
  }
}
  

