// nmpc.hpp ----------------------------------------------------------------
// 4 入力姿勢制御 NMPC (RTI + TR1/SR2 準ニュートン更新 + Box 制約 ADMM)
//
//   状態 x = [phi, theta, p, q, r]^T   (NX = 5)
//   入力 Δu = 姿勢補正 [du_fr, ...]^T (NU = 6, 加算合成: duty = u_hover + Δu)
//   ホライズン N (mpc::N), サンプリング dt (mpc::dt)
//
// 凝縮形:
//   X = Sx * x0 + Su * U_stack
//   J(U) = 0.5 (X - X_ref)^T Q_bar (X - X_ref)
//        + 0.5 (U - U_ref)^T R_bar (U - U_ref)
//
// 解は Hessian H = Su^T Q_bar Su + R_bar、ADMM 用は M = H + rho I を Cholesky
// 維持。TR1 で Su を rank-1 更新、その派生として H/M を rank-2 で外積パッチ
// (rankUpdate(+/-1) を 2 回)。
// -------------------------------------------------------------------------
#ifndef NMPC_HPP
#define NMPC_HPP

#include <Eigen/Dense>
#include <Eigen/Cholesky>
#include "nmpc_params.hpp"
#include "nmpc_model.hpp"

namespace nmpc {

// ---- フル再構築原因 (ベンチ CSV reset_cause 列) ----
enum : uint8_t {
    NMPC_REBUILD_NONE         = 0,
    NMPC_REBUILD_INIT         = 1,
    NMPC_REBUILD_EXTERNAL     = 2,  // nmpc_mark_external_rebuild()
    NMPC_REBUILD_PERIOD       = 3,
    NMPC_REBUILD_SR2_BAD_C    = 4,
    NMPC_REBUILD_SR2_UNHEALTHY = 5,
};

// ---- 直近 step の経路 (ベンチ CSV path 列) ----
enum : uint8_t {
    NMPC_PATH_TR1_SR2          = 0,
    NMPC_PATH_RESET_START      = 1,
    NMPC_PATH_RESET_AFTER_SR2  = 2,
};

// TR1/SR2 / reset 計測 (core1 が更新、core0 ベンチが読む)
struct NmpcStepDiag {
    uint8_t  last_reset_cause;
    uint8_t  last_path;
    uint8_t  last_tr1_applied;
    int      cycles_since_reset;

    uint32_t cnt_tr1_path;
    uint32_t cnt_tr1_applied;
    uint32_t cnt_tr1_skipped;
    uint32_t cnt_reset_period;
    uint32_t cnt_reset_external;
    uint32_t cnt_reset_sr2_bad_c;
    uint32_t cnt_reset_sr2_unhealthy;
    uint32_t cnt_reset_after_sr2;

    // 安全弁の発動回数 (2026-09-09 追加)
    uint32_t cnt_admm_lambda_reset;  // |lambda| 上限で双対をリセットした回数
    uint32_t cnt_nonfinite;          // 解が非有限になり warm-start を破棄した回数
    uint32_t cnt_ulin_reset;         // |U_lin| の暴走を検知して warm-start を畳んだ回数
};

// Stacked types (compile-time fixed sizes)
using XStackVec = Eigen::Matrix<float, mpc::NX_TOTAL, 1>;
using UStackVec = Eigen::Matrix<float, mpc::NU_TOTAL, 1>;

using SxMat = Eigen::Matrix<float, mpc::NX_TOTAL, mpc::NX>;
using SuMat = Eigen::Matrix<float, mpc::NX_TOTAL, mpc::NU_TOTAL>;
using HMat  = Eigen::Matrix<float, mpc::NU_TOTAL, mpc::NU_TOTAL>;

// Box 制約 ADMM の warm-start バッファ
struct WarmStart {
    UStackVec U;       // primal
    UStackVec z;       // 射影変数
    UStackVec lambda;  // dual
    bool      valid;
};

// 関数ローカル一時を BSS に逃がすスクラッチプール (core1 スタック削減)
struct NmpcScratch {
    UStackVec U_lin;
    UStackVec g;
    XStackVec X_nl;
    UStackVec rhs;
    UStackVec admm_neg_c;   // = H*U_lin - g  (= -c)。ADMM の反復間で不変なので 1 回だけ作る
    XStackVec tr1_residual;
    XStackVec tr1_u_rk1;
    UStackVec tr1_s_u;
    XStackVec sr2_Q_u;
    UStackVec sr2_a;
    UStackVec sr2_p;
    UStackVec sr2_q;
    XStackVec grad_dx;
    XStackVec grad_Qdx;
    XStackVec eval_dx;
    UStackVec eval_du;
    SuMat Su_w;
    // モータごとの Δu box (耐故障の η による縮減)。毎周期先頭で作り直す。
    UStackVec du_min;
    UStackVec du_max;
};

// NMPC 状態（呼び出し側で 1 度だけ allocate）
struct NmpcContext {
    // ---- 線形化行列（差分更新で維持） ----
    SxMat Sx;
    SuMat Su;

    // ---- ADMM 用行列  M = H + rho I  ----
    HMat M;
    Eigen::LLT<HMat, Eigen::Lower> llt;  // L L^T = M

    // ---- 前周期スナップショット（割線条件用） ----
    StateVec  x0_prev;
    UStackVec U_lin_prev;
    XStackVec X_lin_prev;     // = Sx_prev * x0_prev + Su_prev * U_lin_prev

    // ---- ADMM warm-start ----
    WarmStart ws;

    // ---- スタック削減用ワークスペース ----
    NmpcScratch scratch;

    // ---- 内部状態 ----
    int  cycles_since_reset;  // フル再構築からの経過周期
    bool need_full_rebuild;   // 数値破綻フラグ → 次周期にフル再構築
    uint8_t rebuild_cause_pending;
    uint8_t  tr1_skip_run;          // TR1 更新を連続で捨てた回数（線形化の鮮度の上限管理）
    uint8_t  ulin_hot_run;         // |U_lin| が上限を超えて連続した周期数（安全弁 3）
    int  last_admm_iters;
    bool initialized;

    // ---- 耐故障: モータ健全度 η (0=故障, 1=健全)。順序 [FR,FL,MR,ML,RR,RL] ----
    // box 制約を Δu ∈ [-u_hover, η*U_duty_max - u_hover] に縮減する。
    // η=0 なら下限=上限=-u_hover となり duty が 0 に固定される。
    float motor_eta[mpc::NU];

    // 直近指標 (デバッグ用)
    float last_cost;
    bool  last_was_reset;
    NmpcStepDiag diag;
};

// 非武装時など: warm-start 破棄 + 次回フル再構築 (原因=EXTERNAL)
void nmpc_mark_external_rebuild(NmpcContext &ctx);

// 遅れ補償: 直近に出した duty 列 hist[0..n-1]（古い順）で x0 を n 周期ぶん前方予測する。
// 実効遅れ（計算 + ZOH + モータ一次遅れ ≈ 40-50 ms）の間に効く指令は既に出したものなので、
// その間の運動をモデルで先回りし、「新しい指令が効き始める時刻」の状態を x0 として与える。
// 非有限になったら x0 をそのまま返す（false）。
bool nmpc_delay_predict(const StateVec &x0, const InputVec *hist, int n, float dt, StateVec &x_pred);

// 耐故障: モータ健全度 η を設定する。eta[j] ∈ [0,1], 0=故障。
// Hessian は box 制約に依存しないため、これを呼んでもフル再構築は起きない。
void nmpc_set_motor_eta(NmpcContext &ctx, const float eta[mpc::NU]);

// 初期化: 全部ゼロ + 必要なフラグセット
void nmpc_init(NmpcContext &ctx);

// core0 で初回フル再構築を先行実行 (core1 初回ピーク回避)
void nmpc_warmup(NmpcContext &ctx);

// メイン関数: 1 周期分の RTI を実行し、最適入力の先頭ブロック U_opt を返す
//
//   x0     : EKF 推定の現在状態 [phi, theta, p, q, r]
//   X_ref  : 参照軌跡 (NX_TOTAL = NX*N)
//   U_ref  : 参照入力 (NU_TOTAL = NU*N)。加算合成では 0 (Δu_ref = 0)
//   u_hover: ホバー baseline duty。Box 制約 Δu ∈ [-u_hover, U_duty_max-u_hover]
//   U_opt  : 出力。最適化結果の先頭 NU 要素 (姿勢補正 Δu)
//   cost_o : 評価値 J(Δu*) を返す
void nmpc_step(NmpcContext &ctx,
               const StateVec &x0,
               const XStackVec &X_ref,
               const UStackVec &U_ref,
               float u_hover,
               InputVec &U_opt,
               float &cost_out);

// ---- ランタイムコスト重み (Wi-Fi / チューニング用) ----
struct NmpcCostParams {
    float Q_phi;
    float Q_theta;
    float Q_p;
    float Q_q;
    float Q_r;
    float R_u;
    float terminal_scale;
};

NmpcCostParams nmpc_cost_defaults();
void nmpc_cost_init();
NmpcCostParams nmpc_cost_get_base();
NmpcCostParams nmpc_cost_get_active();
float nmpc_schedule_alpha(float u_hover);
float nmpc_schedule_alpha_last(void);
NmpcCostParams nmpc_cost_schedule(float u_hover, const NmpcCostParams &base);
void nmpc_cost_set_pending(const NmpcCostParams &p);
bool nmpc_cost_apply_pending(NmpcContext &ctx);
// u_hover に応じた実効重みへ更新 (スケジューリング削除後は no-op)
bool nmpc_cost_update_scheduled(float u_hover, NmpcContext &ctx);

}  // namespace nmpc

#endif  // NMPC_HPP
