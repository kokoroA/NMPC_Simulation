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

#ifdef __cplusplus
}
#endif
#endif
