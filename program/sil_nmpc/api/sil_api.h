// SIL の C ABI（Python が ctypes で叩く）
#ifndef SIL_API_H
#define SIL_API_H
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// --- ライフサイクル -------------------------------------------------------
// ファームの printf を /dev/null へ捨てる（既定は捨てる）。
// control.cpp:958 が 400Hz で Chdata[5] を印字するため、そのままだと洪水になる。
void sil_set_stdout(int enabled);
// 実機 main() の初期化シーケンスを再現し、core1 を起動する
int  sil_init(void);
void sil_shutdown(void);

// --- 入力注入 -------------------------------------------------------------
void sil_set_imu(const float acc_mg[3], const float gyr_mdps[3], const float mag_mgauss[3]);
void sil_set_range_mm(uint16_t mm);
void sil_set_rc(const uint16_t ch[18]);
void sil_set_arm_flag(uint8_t v);

// 自動機動モードを実行時に設定する。
// 実機は MOCK_AUTO_FLIGHT_MODE（コンパイル時）かプロポで入るが、
// SIL では条件を変えて何度も試したいので直接書き込む。
// control.hpp: AUTO_HORIZONTAL_TURN=20, AUTO_FIGURE_EIGHT=21, AUTO_CLIMBING_TURN=22
void sil_set_flight_mode(uint8_t mode);

// PID ゲインを実行時に差し替える（チューニング検討用）。
// 実機は control_init() で固定だが、SIL では同一プロセスで振りたい。
// which: 0=phi 1=theta 2=psi 3=p 4=q 5=r
// h は角度ループ 0.01s / レートループ 0.0025s。0 以下なら既定値を使う。
void sil_set_pid(int which, float kp, float ti, float td, float eta, float h);
// 注: g_thrust_delta_ratio は control.cpp 内の static なので直接読めない。
//     ホスト側で T_ref から逆算する（sim.py の thrust_delta_ratio）。

// --- 実行 -----------------------------------------------------------------
// 400Hz を 1 tick 進める。内部で仮想時間 +2500us、loop_400Hz 呼出、
// core1 が起動された場合はその完了待ちまで行う。
void sil_tick_400hz(void);

// --- 出力 -----------------------------------------------------------------
// [FR, FL, MR, ML, RR, RL] の正規化 duty (0..1)
void sil_get_duty(float duty[6]);
// 姿勢推定・目標値などのスナップショット
void sil_get_state(float out[24]);
uint8_t sil_get_arm_flag(void);
uint8_t sil_get_flight_mode(void);
uint64_t sil_get_time_us(void);

// --- 計算コスト計測 -------------------------------------------------------
// 直近 tick の制御ロジック実行時間（ホスト実測 ns）。
// 実機時間ではない。換算は host 側のコストモデルが行う。
uint64_t sil_get_last_loop_ns(void);
uint64_t sil_get_last_core1_ns(void);

// --- 診断 -----------------------------------------------------------------
// core1 起動回数 / core1 が待ちに入った回数
void sil_get_sched_counts(uint64_t *release_count, uint64_t *acquire_entry_count);
// [Ax,Ay,Az, Mx,My,Mz, Xe0..Xe3, MN,ME,MD, Phi,Theta,Psi, Phi_bias,Theta_bias,Psi_bias, T_ref]
void sil_get_debug(float out[20]);

/* 2026-10-06: Benzen_NMPC 用 */
uint32_t sil_logdata_len(void);
uint32_t sil_logdata_cols(void);
void sil_get_logdata(float *out, uint32_t n);
void sil_get_logdata_row(uint32_t row, float *out);
void sil_set_ftc_target(int motor);   /* FTC で止めるモータ（0..5 = FR..RL, -1 = なし） */
void sil_get_status(uint8_t out[6]);   /* Arm_flag, LockMode, gate_ok, Failsafe_flag, Failsafe_cause, Ftc_mode */

#ifdef __cplusplus
}
#endif
#endif
