// nmpc.cpp ----------------------------------------------------------------
// 4 入力姿勢制御 NMPC の本体実装
//
// 設計概要:
//   ・凝縮形 X = Sx x0 + Su U_stack
//   ・H = Su^T Q_bar Su + R_bar     (Q_bar/R_bar は対角)
//   ・M = H + rho I  を Cholesky 分解 (L L^T) して維持
//   ・周期的にフル再構築 (reset_period); 通常周期は TR1/SR2 で差分更新
//   ・[Sx | Su] を Broyden 風 rank-1 で同時更新
//   ・派生して H/M を rank-2 で外積パッチ (rankUpdate(+/-1) 2 回)
//   ・Box 制約 (0..1) は ADMM (warm-start, RTI:1 反復) で射影
//
// メモリ配置:
//   NmpcContext は呼び出し側で 1 度だけ確保 (~80KB)。
//   ローカルスタック allocation は最小化し、context のスクラッチを再利用。
// -------------------------------------------------------------------------
#include "nmpc.hpp"

#include <math.h>

namespace nmpc {

namespace {

// ---- コスト重み (対角ベクトル化) ----
// Q_bar (NXT 次元): ステージ Q を N-1 個 + 終端 Q_f を 1 個
// R_bar (NUT 次元): ステージ R を N 個
struct CostDiag {
    Eigen::Matrix<float, mpc::NX_TOTAL, 1> qbar;
    Eigen::Matrix<float, mpc::NU_TOTAL, 1> rbar;
};

const CostDiag &cost_diag()
{
    static const CostDiag tab = []() {
        CostDiag c;
        const float Q[mpc::NX] = {
            cost::Q_phi, cost::Q_theta, cost::Q_p, cost::Q_q, cost::Q_r
        };
        const float Qf[mpc::NX] = {
            cost::terminal_scale * cost::Q_phi,
            cost::terminal_scale * cost::Q_theta,
            cost::terminal_scale * cost::Q_p,
            cost::terminal_scale * cost::Q_q,
            cost::terminal_scale * cost::Q_r,
        };
        for (int k = 0; k < mpc::N; ++k) {
            const float *Qk = (k == mpc::N - 1) ? Qf : Q;
            for (int i = 0; i < mpc::NX; ++i) {
                c.qbar(k * mpc::NX + i) = Qk[i];
            }
        }
        for (int k = 0; k < mpc::N; ++k) {
            for (int j = 0; j < mpc::NU; ++j) {
                c.rbar(k * mpc::NU + j) = cost::R_u;
            }
        }
        return c;
    }();
    return tab;
}

// 0.5 * (Sx x0 + Su U - X_ref)^T Q_bar (Sx x0 + Su U - X_ref)
//        + 0.5 * (U - U_ref)^T R_bar (U - U_ref)
float evaluate_cost(const NmpcContext &ctx,
                    const StateVec &x0,
                    const XStackVec &X_ref,
                    const UStackVec &U_ref,
                    const UStackVec &U)
{
    const CostDiag &cd = cost_diag();
    XStackVec dx = ctx.Sx * x0 + ctx.Su * U - X_ref;
    UStackVec du = U - U_ref;
    float jx = 0.5f * dx.cwiseProduct(cd.qbar).dot(dx);
    float ju = 0.5f * du.cwiseProduct(cd.rbar).dot(du);
    return jx + ju;
}

// 線形予測  X_lin = Sx x0 + Su U
void linear_predict(const NmpcContext &ctx,
                    const StateVec &x0,
                    const UStackVec &U,
                    XStackVec &X_lin)
{
    X_lin.noalias() = ctx.Sx * x0 + ctx.Su * U;
}

// 非線形ロールアウト  x_{k+1} = x_k + dt f(x_k, u_k)
void nonlinear_rollout(const StateVec &x0,
                       const UStackVec &U,
                       XStackVec &X_nl)
{
    StateVec x = x0;
    InputVec u_k;
    for (int k = 0; k < mpc::N; ++k) {
        for (int j = 0; j < mpc::NU; ++j) {
            u_k(j) = U(k * mpc::NU + j);
        }
        StateVec x_next;
        simulate_step(x, u_k, x_next);
        for (int i = 0; i < mpc::NX; ++i) {
            X_nl(k * mpc::NX + i) = x_next(i);
        }
        x = x_next;
    }
}

// ---- Sx, Su のフル再構築 (線形化点は (x0, U_lin)) ----
//
//   x_{k+1} = A_d_k x_k + B_d u_k     (B_d は (x,u) 非依存定数)
//   x_{k+1} - x_lin_{k+1} = A_d_k (x_k - x_lin_k) + B_d (u_k - u_lin_k) + 高次
//   凝縮: X = Sx x0 + Su U
//
// 線形化トラジェクトリ (x_lin_k, u_lin_k) を (x0, U_lin) からロールアウトして
// 各時刻の A_d_k を順次評価。
void full_rebuild_sx_su(NmpcContext &ctx,
                        const StateVec &x0,
                        const UStackVec &U_lin)
{
    StateVec x_traj = x0;
    StateMat A_c, A_d;
    InputMat B_c, B_d;
    InputVec u_k;

    ctx.Sx.setZero();
    ctx.Su.setZero();

    // 行ブロック k は x_{k+1} に対応
    //   k=0: Sx[0] = A_d_0 ,  Su[0,0] = B_d
    //   k>0: Sx[k] = A_d_k Sx[k-1] ,  Su[k,j] = A_d_k Su[k-1,j] (j<k), Su[k,k]=B_d
    for (int k = 0; k < mpc::N; ++k) {
        for (int j = 0; j < mpc::NU; ++j) {
            u_k(j) = U_lin(k * mpc::NU + j);
        }
        model_jacobians(x_traj, u_k, A_c, B_c);
        discretize_euler(A_c, B_c, A_d, B_d);

        const int row = k * mpc::NX;
        if (k == 0) {
            ctx.Sx.block<mpc::NX, mpc::NX>(row, 0) = A_d;
            ctx.Su.block<mpc::NX, mpc::NU>(row, 0) = B_d;
        } else {
            const int row_prev = (k - 1) * mpc::NX;
            ctx.Sx.block<mpc::NX, mpc::NX>(row, 0).noalias() =
                A_d * ctx.Sx.block<mpc::NX, mpc::NX>(row_prev, 0);
            for (int j = 0; j < k; ++j) {
                const int col = j * mpc::NU;
                ctx.Su.block<mpc::NX, mpc::NU>(row, col).noalias() =
                    A_d * ctx.Su.block<mpc::NX, mpc::NU>(row_prev, col);
            }
            ctx.Su.block<mpc::NX, mpc::NU>(row, k * mpc::NU) = B_d;
        }

        // x_traj を 1 ステップ進める (非線形)
        StateVec x_next;
        simulate_step(x_traj, u_k, x_next);
        x_traj = x_next;
    }
}

// ---- M = Su^T Q_bar Su + R_bar + rho I を構築し LLT 分解 ----
void full_rebuild_chol(NmpcContext &ctx)
{
    const CostDiag &cd = cost_diag();

    // M = Su^T (Q_bar diag) Su   (重み付き内積)
    // Q_bar が対角なので Q_bar Su は Su の行スケーリング
    // 大きな一時配列を避けるため、要素ごとに評価するループ実装
    ctx.M.setZero();
    // Q_bar^{1/2} * Su を sqrt_Q.asDiagonal() * Su で計算
    // selfadjointView の rankUpdate を使って対称行列構築
    static SuMat Su_w;  // 関数内 static で BSS に確保 (32KB)
    for (int i = 0; i < mpc::NX_TOTAL; ++i) {
        const float sw = sqrtf(cd.qbar(i));
        for (int j = 0; j < mpc::NU_TOTAL; ++j) {
            Su_w(i, j) = sw * ctx.Su(i, j);
        }
    }
    ctx.M.template selfadjointView<Eigen::Lower>().rankUpdate(Su_w.transpose());
    // 対称化 (上三角を埋める)
    ctx.M.template triangularView<Eigen::StrictlyUpper>() =
        ctx.M.transpose().template triangularView<Eigen::StrictlyUpper>();

    // R_bar + rho I を対角に追加
    ctx.M.diagonal() += cd.rbar;
    ctx.M.diagonal().array() += rti::admm_rho;

    // Cholesky 分解
    ctx.llt.compute(ctx.M);
}

// ---- 数値安全性チェック: L 対角が正かどうか ----
bool llt_is_healthy(const Eigen::LLT<HMat, Eigen::Lower> &llt)
{
    if (llt.info() != Eigen::Success) return false;
    const auto &L = llt.matrixLLT();
    for (int i = 0; i < mpc::NU_TOTAL; ++i) {
        if (!(L(i, i) > 1.0e-6f)) return false;
    }
    return true;
}

// ---- TR1: [Sx | Su] を rank-1 (Broyden 風) で更新 ----
//
// 割線: y = X_nl_now - X_lin_prev
//       s = [s_x; s_u] = [x0_now - x0_prev; U_lin_now - U_lin_prev]
//       J s ≈ Sx_old s_x + Su_old s_u
//       residual = y - J s
//       Su を u v^T で更新するため, u = residual / ||s||^2, v_x = s_x, v_u = s_u
//
// 戻り値 update_applied=true なら u, v_u (rank-1 of Su) を out で返す
//   => SR2 の同時 H 更新で利用
struct Tr1Result {
    bool update_applied;
    XStackVec u;        // 残差ベクトル / ||s||^2
    UStackVec v_u;      // s_u 部分 (Su rank-1 の右側)
};

void tr1_update_sx_su(NmpcContext &ctx,
                      const StateVec &x0_now,
                      const UStackVec &U_lin_now,
                      const XStackVec &X_nl_now,
                      Tr1Result &out)
{
    out.update_applied = false;

    StateVec  s_x = x0_now - ctx.x0_prev;
    UStackVec s_u = U_lin_now - ctx.U_lin_prev;

    const float norm_sq = s_x.squaredNorm() + s_u.squaredNorm();
    if (norm_sq < rti::secant_min_norm * rti::secant_min_norm) {
        return;  // ほぼ動いていない → 更新不要
    }

    // y - J s
    XStackVec residual = X_nl_now - ctx.X_lin_prev;
    residual.noalias() -= ctx.Sx * s_x;
    residual.noalias() -= ctx.Su * s_u;

    XStackVec u_rk1 = residual / norm_sq;

    // Sx_new = Sx_old + u_rk1 * s_x^T
    ctx.Sx.noalias() += u_rk1 * s_x.transpose();
    // Su_new = Su_old + u_rk1 * s_u^T
    ctx.Su.noalias() += u_rk1 * s_u.transpose();

    out.update_applied = true;
    out.u   = u_rk1;
    out.v_u = s_u;
}

// ---- SR2: H および M = H + rho I を rank-2 で更新, LLT に rankUpdate ----
//
//   Su_new = Su_old + u v^T   (TR1 で得た更新)
//   ⇒ H_new = H_old + a v^T + v a^T + c v v^T
//      where  a = Su_old^T Q_bar u   (NUT 次元)
//             c = u^T Q_bar u        (スカラー)
//   ⇒ H_new = H_old + p p^T - q q^T
//      where  p = a + (c+1)/2 v
//             q = a + (c-1)/2 v
//
// H は対称正定値が保証される (= Su_new^T Q_bar Su_new + R_bar) が、
// Cholesky の "−" 更新は数値的に脆い。失敗を検出したらフル再構築フラグを立てる。
//
// 注意: tr1_update_sx_su 内で Su は既に更新済み (Su_new)。
//       H 更新で使う a は **Su_old** の項だが、
//         a = (Su_new - u v^T)^T Q_bar u  = Su_new^T Q_bar u - v (u^T Q_bar u)
//                                        = Su_new^T Q_bar u - v c
//       で代用できる。
//
// Powell-style ガード:
//   c = u^T Q_bar u は本来非負だが、数値誤差や TR1 残差の異常で巨大化する場合
//   は更新を見送りフル再構築を要求する。
void sr2_update_h(NmpcContext &ctx,
                  const Tr1Result &tr1)
{
    if (!tr1.update_applied) return;

    const CostDiag &cd = cost_diag();

    XStackVec Q_u = cd.qbar.cwiseProduct(tr1.u);
    UStackVec a   = ctx.Su.transpose() * Q_u;
    const float c = tr1.u.dot(Q_u);
    a.noalias() -= tr1.v_u * c;

    if (!isfinite(c) || fabsf(c) > 1.0e6f) {
        ctx.need_full_rebuild = true;
        return;
    }

    UStackVec p = a + (0.5f * (c + 1.0f)) * tr1.v_u;
    UStackVec q = a + (0.5f * (c - 1.0f)) * tr1.v_u;

    // L L^T を rank-2 で更新 (M を再構築する必要はない: 以後 llt.solve() のみ使う)
    ctx.llt.rankUpdate(p, +1.0f);
    ctx.llt.rankUpdate(q, -1.0f);

    if (!llt_is_healthy(ctx.llt)) {
        ctx.need_full_rebuild = true;
    }
}

// ---- 勾配  g = Su^T Q_bar (Sx x0 - X_ref) - R_bar U_ref ----
void compute_gradient(const NmpcContext &ctx,
                      const StateVec &x0,
                      const XStackVec &X_ref,
                      const UStackVec &U_ref,
                      UStackVec &g)
{
    const CostDiag &cd = cost_diag();
    XStackVec dx = ctx.Sx * x0 - X_ref;
    XStackVec Qdx = cd.qbar.cwiseProduct(dx);
    g.noalias() = ctx.Su.transpose() * Qdx;
    g.noalias() -= cd.rbar.cwiseProduct(U_ref);
}

// ---- ADMM (Box 制約) ----
//
//   min  0.5 U^T H U + g^T U  s.t.  U_min <= U <= U_max
//
// ADMM:
//   U <- (M)^{-1} (-g + rho (z - lambda))    (M = H + rho I, llt で解く)
//   z <- clip(U + lambda, U_min, U_max)
//   lambda <- lambda + U - z
//
// RTI 設定では admm_max_iter=1。warm-start から U/z/lambda を継続使用。
int admm_solve(NmpcContext &ctx,
               const UStackVec &g)
{
    if (!ctx.ws.valid) {
        ctx.ws.U.setZero();
        ctx.ws.z.setZero();
        ctx.ws.lambda.setZero();
        ctx.ws.valid = true;
    }

    const float rho = rti::admm_rho;
    int iters = 0;
    for (; iters < rti::admm_max_iter; ++iters) {
        // U-update
        UStackVec rhs = -g + rho * (ctx.ws.z - ctx.ws.lambda);
        ctx.ws.U = ctx.llt.solve(rhs);

        // z-update (box 射影)
        ctx.ws.z = (ctx.ws.U + ctx.ws.lambda)
                       .cwiseMin(mpc::U_max)
                       .cwiseMax(mpc::U_min);

        // dual update
        ctx.ws.lambda += ctx.ws.U - ctx.ws.z;
    }
    return iters;
}

// ---- warm-start シフト: U[k] <- U[k+1], 末尾は複製 ----
void warm_start_shift(WarmStart &ws)
{
    if (!ws.valid) return;
    auto shift = [](UStackVec &v) {
        for (int k = 0; k < mpc::N - 1; ++k) {
            for (int j = 0; j < mpc::NU; ++j) {
                v(k * mpc::NU + j) = v((k + 1) * mpc::NU + j);
            }
        }
        // 末尾はそのまま (1 周期前の最後をホールド)
    };
    shift(ws.U);
    shift(ws.z);
    shift(ws.lambda);
}

}  // namespace

void nmpc_init(NmpcContext &ctx)
{
    ctx.Sx.setZero();
    ctx.Su.setZero();
    ctx.M.setZero();
    ctx.x0_prev.setZero();
    ctx.U_lin_prev.setZero();
    ctx.X_lin_prev.setZero();
    ctx.ws.U.setZero();
    ctx.ws.z.setZero();
    ctx.ws.lambda.setZero();
    ctx.ws.valid = false;
    ctx.cycles_since_reset = 0;
    ctx.need_full_rebuild = true;  // 初回はフル再構築
    ctx.last_admm_iters = 0;
    ctx.initialized = true;
    ctx.last_cost = 0.0f;
    ctx.last_was_reset = false;
}

void nmpc_step(NmpcContext &ctx,
               const StateVec &x0,
               const XStackVec &X_ref,
               const UStackVec &U_ref,
               InputVec &U_opt,
               float &cost_out)
{
    if (!ctx.initialized) {
        nmpc_init(ctx);
    }

    // 1) warm-start シフト → 線形化軌跡 U_lin として使用
    //    U_lin は ws.U を **コピー** しておく (ADMM が後で ws.U を上書きするため)
    warm_start_shift(ctx.ws);
    UStackVec U_lin = ctx.ws.U;  // ws.valid==false なら全ゼロ (init で setZero 済)

    // 2) フル再構築 or 差分更新
    bool reset_now = ctx.need_full_rebuild
                  || (ctx.cycles_since_reset >= rti::reset_period);

    if (reset_now) {
        full_rebuild_sx_su(ctx, x0, U_lin);
        full_rebuild_chol(ctx);
        ctx.cycles_since_reset = 0;
        ctx.need_full_rebuild = false;
        ctx.last_was_reset = true;
    } else {
        // 非線形ロールアウト → TR1 → SR2
        XStackVec X_nl;
        nonlinear_rollout(x0, U_lin, X_nl);

        Tr1Result tr1;
        tr1_update_sx_su(ctx, x0, U_lin, X_nl, tr1);
        sr2_update_h(ctx, tr1);

        ctx.cycles_since_reset += 1;
        ctx.last_was_reset = false;

        if (ctx.need_full_rebuild) {
            // 数値破綻が検出されたので即座にフル再構築
            full_rebuild_sx_su(ctx, x0, U_lin);
            full_rebuild_chol(ctx);
            ctx.cycles_since_reset = 0;
            ctx.need_full_rebuild = false;
            ctx.last_was_reset = true;
        }
    }

    // 3) 勾配計算
    UStackVec g;
    compute_gradient(ctx, x0, X_ref, U_ref, g);

    // 4) ADMM
    ctx.last_admm_iters = admm_solve(ctx, g);

    // 5) 結果取り出し: 先頭 NU 個 (現周期に印加)
    for (int j = 0; j < mpc::NU; ++j) {
        float v = ctx.ws.z(j);  // 射影変数を出力 (制約必ず満たす)
        if (v < mpc::U_min) v = mpc::U_min;
        if (v > mpc::U_max) v = mpc::U_max;
        U_opt(j) = v;
    }

    // 6) コスト評価 (デバッグ用)
    cost_out = evaluate_cost(ctx, x0, X_ref, U_ref, ctx.ws.z);
    ctx.last_cost = cost_out;

    // 7) 次周期用スナップショット保存
    ctx.x0_prev = x0;
    ctx.U_lin_prev = U_lin;
    linear_predict(ctx, x0, U_lin, ctx.X_lin_prev);
}

}  // namespace nmpc
