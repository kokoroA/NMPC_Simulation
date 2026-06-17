// nmpc_params.hpp ---------------------------------------------------------
// 機体・NMPC ハイパーパラメータ集約
//
// 機体を変えたら physical:: を、チューニングは cost:: を、
// 予測ホライズン等は mpc:: を書き換える。
//
// 全て constexpr のためコンパイル時に評価され、ROM/RAM 消費は無し
// （実際にはコード内に即値として展開される）。
// -------------------------------------------------------------------------
#ifndef NMPC_PARAMS_HPP
#define NMPC_PARAMS_HPP

namespace nmpc {

// =========================================================================
// 1. 物理パラメータ（機体ごとに調整）
// =========================================================================
namespace physical {
    // 慣性モーメント [kg*m^2]
    constexpr float Ix = 6.10e-3f;
    constexpr float Iy = 6.53e-3f;
    constexpr float Iz = 1.16e-2f;

    // 空力ダンピング係数（角速度^2 への比例係数）
    constexpr float Cp = 0.01f;
    constexpr float Cq = 0.025f;
    constexpr float Cr = 0.005f;

    // 機体形状
    constexpr float arm_length = 0.050f;   // l [m]   中心-モータ距離
    constexpr float mass       = 0.200f;   // [kg]
    constexpr float gravity    = 9.81f;    // [m/s^2]

    // プロペラ係数（推力/反トルク）
    constexpr float Ct = 8.3e-7f;          // 推力係数  [N*s^2]
    constexpr float CQ = 3.0e-8f;          // 反トルク係数 [N*m*s^2]

    // 反トルク／推力比 [m] (符号 +) — モーターミキサで使用
    constexpr float C_QT = CQ / Ct;        // ≈ 0.03614

    // ホバー時 1 モータ推力 [N]
    constexpr float T_hover_per_motor = mass * gravity / 4.0f; // ≈ 0.4905

    // モータ最大推力 [N] (duty=1 時の 1 モータ推力)
    // ※暫定値：実機ホバー測定後に T_hover_per_motor / hover_duty で更新推奨
    constexpr float T_max = 2.725f;
}

// =========================================================================
// 2. NMPC ハイパーパラメータ
// =========================================================================
namespace mpc {
    constexpr int   N    = 20;             // 予測ホライズン
    constexpr float dt   = 0.01f;          // サンプリング [s] (100Hz)

    constexpr int   NX   = 5;              // 状態次元 [phi, theta, p, q, r]
    constexpr int   NU   = 4;              // 入力次元 [u_fr, u_fl, u_rr, u_rl]

    // 予測軌跡サイズ
    constexpr int   NX_TOTAL = NX * N;     // = 100
    constexpr int   NU_TOTAL = NU * N;     // = 80

    // 入力制約 [duty]
    constexpr float U_min = 0.0f;
    constexpr float U_max = 1.0f;

    // 数値ダンピング微小量（|p|→sqrt(p^2+eps^2) 平滑化用）
    constexpr float damping_eps = 1.0e-3f;
}

// =========================================================================
// 3. コスト関数重み（Bryson's Rule 初期値）
// =========================================================================
namespace cost {
    // 状態許容偏差
    constexpr float dev_phi   = 0.10f;     // [rad]
    constexpr float dev_theta = 0.10f;
    constexpr float dev_p     = 1.0f;      // [rad/s]
    constexpr float dev_q     = 1.0f;
    constexpr float dev_r     = 1.0f;

    // 入力許容偏差 [duty]
    constexpr float dev_u     = 0.20f;

    // ステージコスト Q 対角（Bryson: 1/dev^2）
    constexpr float Q_phi   = 1.0f / (dev_phi   * dev_phi);    // 100
    constexpr float Q_theta = 1.0f / (dev_theta * dev_theta);  // 100
    constexpr float Q_p     = 1.0f / (dev_p     * dev_p);      // 1
    constexpr float Q_q     = 1.0f / (dev_q     * dev_q);      // 1
    constexpr float Q_r     = 1.0f / (dev_r     * dev_r);      // 1

    // 入力コスト R 対角
    constexpr float R_u = 1.0f / (dev_u * dev_u);              // 25

    // 終端コスト倍率
    constexpr float terminal_scale = 5.0f;
}

// =========================================================================
// 4. RTI / 準ニュートン法の制御
// =========================================================================
namespace rti {
    // フル再構築（リセット）周期 [周期数]
    // 差分更新の数値誤差累積対策。
    // 1 = 毎周期フル再構築（デバッグ用、TR1/SR2 をバイパス）
    constexpr int   reset_period = 100;

    // SR2 ダンピング下限（Powell-style）
    // s^T*y / (s^T*B*s) がこの値を下回ると割線条件を緩める
    constexpr float sr2_damping_min = 0.2f;

    // 割線条件の最小ノルム（過小な s/y では更新スキップ）
    constexpr float secant_min_norm = 1.0e-6f;

    // ADMM
    // RTI: 1 反復のみ。ただし初期段階では収束を見るため複数反復可能。
    constexpr int   admm_max_iter = 1;
    constexpr float admm_rho      = 1.0f;
}

}  // namespace nmpc

#endif  // NMPC_PARAMS_HPP
