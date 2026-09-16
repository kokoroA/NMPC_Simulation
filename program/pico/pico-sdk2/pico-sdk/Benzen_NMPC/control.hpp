#ifndef CONTROL_HPP
#define CONTROL_HPP

#include <stdio.h>
#include "pico_copter.hpp"
#include "pico/stdlib.h"
#include "sensor.hpp"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include <Eigen/Dense>
#include "ekf.hpp"
#include <math.h>
#include "rgbled.hpp"
#include "pid.hpp"

using Eigen::MatrixXd;
using Eigen::MatrixXf;
using Eigen::Matrix;
using Eigen::PartialPivLU;
using namespace Eigen;

// #define BATTERY_VOLTAGE (11.1)
#define BATTERY_VOLTAGE (7.4)
// ---- Flight_mode の値 ----
// ⚠ radio.hpp のチャンネル番号 (FTC=5, HOVERING=8 等) とは別物。混ぜないこと。
//    以前 REDCIRCLE がチャンネル番号と Flight_mode 値の両方に使われており、
//    radio.hpp 側の名前が変わった瞬間にビルドが壊れた。同じ轍を踏まないこと。
#define NORMAL        0
#define MODE_HOVERING 1
#define MODE_FTC      2
#define BUFFER_SIZE 4

// ---- モータ番号 (Delta_opt / Motor_eta の添字) ----
#define MOTOR_FR 0
#define MOTOR_FL 1
#define MOTOR_MR 2
#define MOTOR_ML 3
#define MOTOR_RR 4
#define MOTOR_RL 5
#define MOTOR_NONE (-1)
#define NUM_MOTOR 6


//グローバル関数の宣言
void loop_400Hz(void);
void control_init();
void rate_control(void);
void angle_control(void);
void gyro_calibration(void);
void variable_init(void);
void log_output(void);
void rgbled_nomal(void);
void rgbled_off(void);
void debug_print_gyro(void);
void debug_print_nmpc_bench(void);
void debug_poll_pending_messages(void);

//グローバル変数
// ---- 耐故障(FTC)モード ----
// Ftc_target_motor: 停止させるモータ。★ここを書き換えるだけで対象を変更できる★
//                   MOTOR_FR / MOTOR_FL / MOTOR_MR / MOTOR_ML / MOTOR_RR / MOTOR_RL
//                   MOTOR_NONE にすると FTC スイッチを入れても何も止めない。
// Motor_eta      : モータ健全度 [0,1]。0=故障, 1=健全。順序 [FR,FL,MR,ML,RR,RL]
//                  NMPC の box 制約縮減と U_ref シフトはこの配列だけを見る。
extern uint8_t Ftc_mode;
extern int8_t  Ftc_target_motor;
extern float   Motor_eta[NUM_MOTOR];
// ---- フェイルセーフ / 脱出条件 ----
extern uint8_t Failsafe_flag;   // 1 = ラッチ。解除は Arm_flag を 2 以外にするまで
extern uint8_t Failsafe_cause;  // FS_* のいずれか
#define FS_NONE      0
// 1 は旧 FS_RX_LOST (0x04 と 0x08 を区別せず単発で発動していた)。
// どちらが起きたのかログから判別できず原因特定を遅らせたため 6/7 に分離した。
// 過去ログとの互換のため番号だけ空けておき、新規には使わない。
#define FS_RX_LOST   1  // 廃止。使用禁止
#define FS_RX_TIMEOUT 2 // 受信途絶 (200ms フレームが来ない)
#define FS_ATTITUDE  3  // |phi| or |theta| 超過
#define FS_RATE      4  // |p| or |q| or |r| 超過
#define FS_OVERG     5  // 加速度超過
#define FS_RX_FAILSAFE  6 // SBUS 0x08: 受信機が送信機を見失った (本物。即発動)
#define FS_RX_FRAMELOST 7 // SBUS 0x04 が連続 (単発では発動しない)

// ---- ベンチ加振モード (システム同定用) ----
// NMPC を通さずに duty を直接書く。閉ループ共線性・50Hz チャタリング・
// モータ遅れ未同定という飛行ログ回帰の敗因 3 つを同時に消すのが目的。
// 手順の全文: docs/20260905_yaw_damping_experiment_protocol.md
#define BENCH_OFF        0  // 通常飛行
#define BENCH_YAW_EXCITE 1  // ヨー差動加振 (C_QT/Iz, b/Iz, Cr/Iz, モータ遅れ)
#define BENCH_MOTORS_OFF 2  // モータ停止のままログだけ取る (フェーズ0 ベースライン)
extern uint8_t Bench_mode;
extern float   Bench_u0;       // common duty
extern float   Bench_delta;    // 差動量
extern float   Bench_sign;     // 差動の向き (+1 / -1)
extern float   Bench_u_start;  // ランプ開始 duty
extern float   Bench_t_ramp;   // (1a) u_start -> u0 のランプ [s]
extern float   Bench_t_spin;   // (1b) u0 保持 [s]
extern float   Bench_t_step;   // (2)(3) 差動投入 [s]
extern float   Bench_t_decay;  // (4)   自由減衰 [s]
extern float   Bench_r_max;    // |r| の上限 [deg/s]。超えたら (2) を打ち切る。0=無効
extern uint8_t Bench_phase;    // 0=待機 1=ramp/spin 2=step 3=decay 4=終了

extern uint8_t LockMode;
extern volatile uint8_t Logoutputflag;
extern float ideal;
extern float input;
extern float lotated_distance;
extern float z_acc;
extern float func_time;
extern float T_ref;
extern uint64_t count_up;



//グローバル変数
extern uint8_t LockMode;
extern volatile uint8_t Logoutputflag;


// class PID
// {
//   private:
//     float m_kp;
//     float m_ti;
//     float m_td;
//     float m_filter_time_constant;
//     float m_err,m_err2,m_err3;
//     float m_h;
//   public:
//     float m_filter_output;
//     float m_integral;
//     PID();
//     void set_parameter(
//         float kp, 
//         float ti, 
//         float td,
//         float filter_time_constant, 
//         float h);
//     void reset(void);
//     void i_reset(void);
//     void printGain(void);
//     float filter(float x);
//     float update(float err);
// };

// class Filter
// {
//   private:
//     float m_state;
//     float m_T;
//     float m_h;
//   public:
//     float m_out;
//     Filter();
//     void set_parameter(
//         float T,
//         float h);
//     void reset(void);
//     float update(float u);
// };


#endif
