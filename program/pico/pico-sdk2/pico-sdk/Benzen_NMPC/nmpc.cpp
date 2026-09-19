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
//   ・Box 制約 (0..U_duty_max) は ADMM (warm-start, RTI:1 反復) で射影
//
// メモリ配置:
//   NmpcContext + scratch は呼び出し側で 1 度だけ BSS 確保。
//   大きな一時配列は ctx.scratch に集約し core1 スタック消費を最小化。
// -------------------------------------------------------------------------
#include "nmpc.hpp"

#include <math.h>

namespace nmpc {

NmpcCostParams nmpc_cost_defaults();

// ---- ランタイムコスト重み (匿名 namespace 外: Wi-Fi API からも参照) ----
struct CostDiag {
    Eigen::Matrix<float, mpc::NX_TOTAL, 1> qbar;
    Eigen::Matrix<float, mpc::NU_TOTAL, 1> rbar;
};

static volatile uint8_t g_cost_pending_flag;
static NmpcCostParams g_cost_pending;
static NmpcCostParams g_cost_base;
static NmpcCostParams g_cost_active;
static CostDiag g_cost_diag;
static bool g_cost_inited = false;

static float clamp_cost(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static NmpcCostParams clamp_cost_params(const NmpcCostParams &p)
{
    NmpcCostParams out = p;
    out.Q_phi = clamp_cost(p.Q_phi, 0.1f, 1000.0f);
    out.Q_theta = clamp_cost(p.Q_theta, 0.1f, 1000.0f);
    out.Q_p = clamp_cost(p.Q_p, 0.1f, 1000.0f);
    out.Q_q = clamp_cost(p.Q_q, 0.1f, 1000.0f);
    out.Q_r = clamp_cost(p.Q_r, 0.1f, 1000.0f);
    out.R_u = clamp_cost(p.R_u, 0.1f, 100.0f);
    out.terminal_scale = clamp_cost(p.terminal_scale, 1.0f, 100.0f);
    return out;
}

static void build_cost_diag(CostDiag &cd, const NmpcCostParams &p)
{
    const float Q[mpc::NX] = { p.Q_phi, p.Q_theta, p.Q_p, p.Q_q, p.Q_r };
    const float Qf[mpc::NX] = {
        p.terminal_scale * p.Q_phi,
        p.terminal_scale * p.Q_theta,
        p.terminal_scale * p.Q_p,
        p.terminal_scale * p.Q_q,
        p.terminal_scale * p.Q_r,
    };
    for (int k = 0; k < mpc::N; ++k) {
        const float *Qk = (k == mpc::N - 1) ? Qf : Q;
        for (int i = 0; i < mpc::NX; ++i) {
            cd.qbar(k * mpc::NX + i) = Qk[i];
        }
    }
    for (int k = 0; k < mpc::N; ++k) {
        for (int j = 0; j < mpc::NU; ++j) {
            cd.rbar(k * mpc::NU + j) = p.R_u;
        }
    }
}

static const CostDiag &cost_diag_ref()
{
    if (!g_cost_inited) {
        g_cost_base = nmpc_cost_defaults();
        g_cost_active = g_cost_base;
        build_cost_diag(g_cost_diag, g_cost_active);
        g_cost_inited = true;
    }
    return g_cost_diag;
}

namespace {

const CostDiag &cost_diag()
{
    return cost_diag_ref();
}

#if NMPC_BENCH_COST
float evaluate_cost(NmpcContext &ctx,
                    const StateVec &x0,
                    const XStackVec &X_ref,
                    const UStackVec &U_ref,
                    const UStackVec &U)
{
    const CostDiag &cd = cost_diag();
    ctx.scratch.eval_dx.noalias() = ctx.Sx * x0 + ctx.Su * U - X_ref;
    ctx.scratch.eval_du = U - U_ref;
    float jx = 0.5f * ctx.scratch.eval_dx.cwiseProduct(cd.qbar).dot(ctx.scratch.eval_dx);
    float ju = 0.5f * ctx.scratch.eval_du.cwiseProduct(cd.rbar).dot(ctx.scratch.eval_du);
    return jx + ju;
}
#endif

void bump_rebuild_counter(NmpcStepDiag &d, uint8_t cause)
{
    switch (cause) {
    case NMPC_REBUILD_PERIOD:
        d.cnt_reset_period++;
        break;
    case NMPC_REBUILD_EXTERNAL:
        d.cnt_reset_external++;
        break;
    case NMPC_REBUILD_SR2_BAD_C:
        d.cnt_reset_sr2_bad_c++;
        break;
    case NMPC_REBUILD_SR2_UNHEALTHY:
        d.cnt_reset_sr2_unhealthy++;
        break;
    default:
        break;
    }
}

void linear_predict(const NmpcContext &ctx,
                    const StateVec &x0,
                    const UStackVec &U,
                    XStackVec &X_lin)
{
    X_lin.noalias() = ctx.Sx * x0 + ctx.Su * U;
}

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

bool llt_is_healthy(const Eigen::LLT<HMat, Eigen::Lower> &llt);

// M = Su_w^T * Su_w を BSS 上の M に直接書き込む (rankUpdate / Eigen GEMM のスタック消費を回避)
// ---- 2026-09-19: Su のブロック下三角構造を使う ----
// full_rebuild_sx_su() は Su.setZero() のあと j <= k のブロックにしか書かないので、
// **入力 i が属する段 ki = i/NU より前の行（k < ki*NX）は厳密にゼロ**。
// j <= i なら kj <= ki なので、内側ループは k = ki*NX から始めてよい。
//   積和回数 249,900 -> 92,925 (-62.8%)。**足していた項が厳密に 0 なので結果はビット単位で同一。**
// ⚠ この関数は full_rebuild_chol() からしか呼ばれず、その直前に必ず full_rebuild_sx_su() が
//   走るので、Su は常にブロック下三角。**TR1 のランク 1 更新後の Su に対して呼ばないこと**
//   （TR1 は上側ブロックを埋めうるため、この最適化が成立しなくなる）。
void build_hessian_from_su_w(const SuMat &Su_w, HMat &M)
{
    for (int i = 0; i < mpc::NU_TOTAL; ++i) {
        const int k0 = (i / mpc::NU) * mpc::NX;   // これより前の行は Su_w が厳密に 0
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = k0; k < mpc::NX_TOTAL; ++k) {
                sum += Su_w(k, i) * Su_w(k, j);
            }
            M(i, j) = sum;
            if (i != j) {
                M(j, i) = sum;
            }
        }
    }
}

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

        StateVec x_next;
        simulate_step(x_traj, u_k, x_next);
        x_traj = x_next;
    }
}

void full_rebuild_chol(NmpcContext &ctx)
{
    const CostDiag &cd = cost_diag();

    ctx.M.setZero();
    SuMat &Su_w = ctx.scratch.Su_w;
    for (int i = 0; i < mpc::NX_TOTAL; ++i) {
        const float sw = sqrtf(cd.qbar(i));
        for (int j = 0; j < mpc::NU_TOTAL; ++j) {
            Su_w(i, j) = sw * ctx.Su(i, j);
        }
    }

    build_hessian_from_su_w(Su_w, ctx.M);

    ctx.M.diagonal() += cd.rbar;
    ctx.M.diagonal().array() += rti::admm_rho;

    ctx.llt.compute(ctx.M);
}

bool llt_is_healthy(const Eigen::LLT<HMat, Eigen::Lower> &llt)
{
    if (llt.info() != Eigen::Success) return false;
    const auto &L = llt.matrixLLT();
    for (int i = 0; i < mpc::NU_TOTAL; ++i) {
        if (!(L(i, i) > 1.0e-6f)) return false;
    }
    return true;
}

bool tr1_update_sx_su(NmpcContext &ctx,
                      const StateVec &x0_now,
                      const UStackVec &U_lin_now,
                      const XStackVec &X_nl_now)
{
    StateVec s_x = x0_now - ctx.x0_prev;
    ctx.scratch.tr1_s_u = U_lin_now - ctx.U_lin_prev;

    const float norm_sq = s_x.squaredNorm() + ctx.scratch.tr1_s_u.squaredNorm();
    ctx.diag.last_secant_norm_sq = norm_sq;          // debug (2026-09-19)
    if (norm_sq < rti::secant_min_norm * rti::secant_min_norm) {
        ctx.diag.last_tr1_c = 0.0f;                  // c は計算していない
        ctx.diag.last_tr1_skip_run = ctx.tr1_skip_run;
        return false;
    }

    ctx.scratch.tr1_residual = X_nl_now - ctx.X_lin_prev;
    ctx.scratch.tr1_residual.noalias() -= ctx.Sx * s_x;
    ctx.scratch.tr1_residual.noalias() -= ctx.Su * ctx.scratch.tr1_s_u;

    ctx.scratch.tr1_u_rk1 = ctx.scratch.tr1_residual / norm_sq;

    // ---- 割線更新の大きさを Su/Sx へ反映する前に検査する（2026-09-10）----
    // c = u_rk1^T Q u_rk1 は SR2 のランク 2 更新の大きさそのもの。これが 1e6 を超えると
    // 従来は sr2_update_h() が「SR2_BAD_C」として**その周期の中で 36 ms のフル再構築**を
    // 走らせていた。0910_100Hz_v4 では飛行中に 80 回（約 5 回/秒）発火し、
    // 墜落直前 1.25 秒の 52% が duty ホールド状態だった（R_u=320 より R_u=800 で 2 倍増える）。
    // 36 ms の無制御は 40-50 ms の実効遅れを持つ 4 Hz 限界モードに致命的で、
    // 「ヘシアンが 1 周期古い」害より桁違いに大きい。
    // → 大きすぎる更新は **適用せずに捨てる**（Powell 流のスキップ）。Su/Sx/LLT は前周期の
    //   整合した組のまま残るので、定数項 c と M の整合も保たれる。再構築は周期リセットに任せる。
    // 詳細: docs/ai/decisions/20260910_sr2_bad_c_rebuild_storm.md
    {
        const CostDiag &cd = cost_diag();
        const float c = ctx.scratch.tr1_u_rk1.dot(cd.qbar.cwiseProduct(ctx.scratch.tr1_u_rk1));
        ctx.diag.last_tr1_c = c;                     // debug (2026-09-19)
        if (!(fabsf(c) <= rti::tr1_c_max)) {   // NaN も捕まえる
            ctx.diag.cnt_reset_sr2_bad_c++;   // 名前は歴史的経緯。「大きすぎて捨てた回数」を数える
            // 捨て続けると Su/Sx が古いまま状態だけ進み、残差が大きいので更に捨てる、という
            // 罠に入る（リプレイで最大 27 周期 = 0.27 s 連続）。鮮度の上限として、
            // 連続 tr1_skip_max_run 回で 1 回だけフル再構築を要求する。
            if (++ctx.tr1_skip_run >= rti::tr1_skip_max_run) {
                ctx.tr1_skip_run = 0;
                ctx.need_full_rebuild = true;
                ctx.rebuild_cause_pending = NMPC_REBUILD_SR2_BAD_C;
                ctx.diag.last_tr1_skip_run = rti::tr1_skip_max_run;  // 到達した回数を残す
            } else {
                ctx.diag.last_tr1_skip_run = ctx.tr1_skip_run;
            }
            return false;
        }
        ctx.tr1_skip_run = 0;
        ctx.diag.last_tr1_skip_run = 0;
    }

    ctx.Sx.noalias() += ctx.scratch.tr1_u_rk1 * s_x.transpose();
    ctx.Su.noalias() += ctx.scratch.tr1_u_rk1 * ctx.scratch.tr1_s_u.transpose();

    return true;
}

void sr2_update_h(NmpcContext &ctx, bool tr1_applied)
{
    if (!tr1_applied) return;

    const CostDiag &cd = cost_diag();

    ctx.scratch.sr2_Q_u = cd.qbar.cwiseProduct(ctx.scratch.tr1_u_rk1);
    ctx.scratch.sr2_a.noalias() = ctx.Su.transpose() * ctx.scratch.sr2_Q_u;
    const float c = ctx.scratch.tr1_u_rk1.dot(ctx.scratch.sr2_Q_u);
    ctx.scratch.sr2_a.noalias() -= ctx.scratch.tr1_s_u * c;

    // tr1_update_sx_su() が同じ c を先に検査して大きい更新を捨てるので、ここへは
    // 原則 |c| <= tr1_c_max のものしか来ない。最後の砦として NaN だけは再構築へ回す。
    if (!isfinite(c)) {
        ctx.need_full_rebuild = true;
        ctx.rebuild_cause_pending = NMPC_REBUILD_SR2_BAD_C;
        return;
    }

    ctx.scratch.sr2_p = ctx.scratch.sr2_a + (0.5f * (c + 1.0f)) * ctx.scratch.tr1_s_u;
    ctx.scratch.sr2_q = ctx.scratch.sr2_a + (0.5f * (c - 1.0f)) * ctx.scratch.tr1_s_u;

    ctx.llt.rankUpdate(ctx.scratch.sr2_p, +1.0f);
    ctx.llt.rankUpdate(ctx.scratch.sr2_q, -1.0f);

    if (!llt_is_healthy(ctx.llt)) {
        ctx.need_full_rebuild = true;
        ctx.rebuild_cause_pending = NMPC_REBUILD_SR2_UNHEALTHY;
    }
}

// ADMM の U 更新 (H + rho I) U = -c + rho (z - lambda) に必要なのは
// 「U_lin における勾配 g」ではなく「コストの定数項 c」である。
//
//   J(U) = 0.5 (Sx x0 + Su U - X_ref)^T Q (...) + 0.5 (U - U_ref)^T R (...)
//   ∇J(U) = H U + c,  H = Su^T Q Su + R,  c = Su^T Q (Sx x0 - X_ref) - R U_ref
//
// c は U に依存しない。定義どおり組み立てれば U_lin は一切現れない。
//
// ⚠ 2026-09-09: これを g から逆算して c = H*U_lin - g と作っていたことがあり、
//   離陸数秒で box 隅まで暴走して墜落した（0909_v5 / 0909_100Hz_v2）。
//   逆算に使う H は TR1/SR2 の準ニュートン近似、g は現在の Su から作った厳密勾配なので、
//   両者は別物である。差は (H_SR2 - H_exact)·U_lin となり、**U_lin に比例して増幅される**。
//   ホスト再現: 記録された飛行状態を再入力すると右辺の相対誤差は平均 21.5% / 最大 740%、
//   線形化点 |U_lin| は物理上限 (√120×0.95 = 10.4) を大きく超える 352 まで発散した。
//   毎周期フル再構築で H を厳密化すると誤差 3.2% / |U_lin| 2.1 に落ちることから、
//   原因が「H と g の不整合」であることを確認済み。
//   詳細: docs/ai/decisions/20260909_admm_const_term_backsolve_divergence.md
void compute_neg_const_term(NmpcContext &ctx,
                            const StateVec &x0,
                            const XStackVec &X_ref,
                            const UStackVec &U_ref)
{
    const CostDiag &cd = cost_diag();
    ctx.scratch.grad_dx.noalias() = ctx.Sx * x0 - X_ref;
    ctx.scratch.grad_Qdx = -(cd.qbar.cwiseProduct(ctx.scratch.grad_dx));
    ctx.scratch.admm_neg_c.noalias() = ctx.Su.transpose() * ctx.scratch.grad_Qdx;
    ctx.scratch.admm_neg_c += cd.rbar.cwiseProduct(U_ref);   // = -c
}

#if NMPC_LEGACY_SOLVER
// ---- 旧ソルバ再現（2026-09-10、切替式）----
// 2026-09-09 以前の実装: ADMM の右辺に「U_lin における完全な勾配 g = H*U_lin + c」をそのまま使う。
// これは U_new ≈ -U_lin - H^{-1}c という固有値 -1 の写像で、不動点は正解の半分、
// 毎周期符号が反転する 50 Hz ディザを出す（docs/ai/decisions/20260909_admm_gradient_bug_root_cause.md）。
// 0904 の D90S 飛行はこの状態で飛べていたため、GEMFAN でも飛べるかを A/B するための再現モード。
// 安全弁（λ 上限・非有限検出・EKF ガード）と TR1 の過大更新棄却はそのまま残す。
void compute_neg_gradient_legacy(NmpcContext &ctx,
                                 const StateVec &x0,
                                 const XStackVec &X_ref,
                                 const UStackVec &U_ref,
                                 const UStackVec &U_lin)
{
    const CostDiag &cd = cost_diag();
    // 勾配を評価する点だけ γ·U_lin にする → 写像 U_new = -γ U_lin - H^{-1}c（固有値 -γ）。
    const float g_ = rti::legacy_gamma;
    ctx.scratch.grad_dx.noalias() = ctx.Sx * x0 - X_ref;
    ctx.scratch.grad_dx.noalias() += ctx.Su * (g_ * U_lin);
    ctx.scratch.grad_Qdx = -(cd.qbar.cwiseProduct(ctx.scratch.grad_dx));
    ctx.scratch.admm_neg_c.noalias() = ctx.Su.transpose() * ctx.scratch.grad_Qdx;   // = -Su^T Q (…)
    ctx.scratch.eval_du = g_ * U_lin - U_ref;
    ctx.scratch.admm_neg_c -= cd.rbar.cwiseProduct(ctx.scratch.eval_du);            // = -g(γ U_lin)
}
#endif

// 加算合成 duty = u_hover + Δu の実機クリップ [0, eta*U_duty_max] と一致する Δu 範囲。
// eta = 1.0 で従来どおり。eta = 0.0 なら lo == hi == -u_hover となり duty が 0 に固定される。
inline void delta_u_bounds_eta(float u_hover, float eta, float &du_min, float &du_max)
{
    du_min = -u_hover;
    du_max = eta * mpc::U_duty_max - u_hover;
    if (du_max < du_min) {
        du_max = du_min;
    }
}

// ctx.motor_eta から Δu の box をモータごとに展開する。
// ホライズン全段に同じ値を敷く（故障はホライズン内で変化しない前提）。
inline void delta_u_bounds_fill(NmpcContext &ctx, float u_hover)
{
    for (int j = 0; j < mpc::NU; ++j) {
        float lo = 0.0f, hi = 0.0f;
        delta_u_bounds_eta(u_hover, ctx.motor_eta[j], lo, hi);
        for (int k = 0; k < mpc::N; ++k) {
            ctx.scratch.du_min(k * mpc::NU + j) = lo;
            ctx.scratch.du_max(k * mpc::NU + j) = hi;
        }
    }
}

// warm start を捨てたときの再起動点。
// ---- 2026-09-15: 原点ではなく **U_ref** へ戻す ----
// 旧実装は全部 0 に落としていた。健全時は U_ref = 0 なので原点でよいが、耐故障時の
// Δu = 0 は「健全 5 発すべて u_hover」＝ **故障を忘れた配分**であり、ヨートルクが
// Σk_S·u = +u_hover（= +194 deg/s²）残る。0915_100Hz_v1 では 39 ms のフル再構築中に
// これが ZOH で保持され、飛行時間の 50% を占めてヨーが +221 deg/s まで発散した。
// U_ref は 3 軸のモーメントが厳密に釣り合う配分なので、フォールバック先として正しい。
// ⚠ 健全時は U_ref = 0 かつ du_min < 0 < du_max なので **旧実装とビット一致**する。
inline void warm_start_reset_to_ref(NmpcContext &ctx, const UStackVec &U_ref)
{
    ctx.ws.z = U_ref.cwiseMin(ctx.scratch.du_max).cwiseMax(ctx.scratch.du_min);
    ctx.ws.U = ctx.ws.z;
    ctx.ws.lambda.setZero();
}

int admm_solve(NmpcContext &ctx, float u_hover, const UStackVec &U_ref)
{
    // box はモータごと（耐故障の η で FR 等を 0 に固定できる）。
    // ⚠ warm_start_reset_to_ref() が du_min/du_max を使うので、box を先に張る。
    delta_u_bounds_fill(ctx, u_hover);

    if (!ctx.ws.valid) {
        warm_start_reset_to_ref(ctx, U_ref);
        ctx.ws.valid = true;
    }
    const float rho = rti::admm_rho;

    // ctx.scratch.admm_neg_c には compute_neg_const_term() が入れた -c が入っている。
    // 反復間で不変なので、ループの外で 1 回作れば足りる。

    int iters = 0;
    for (; iters < rti::admm_max_iter; ++iters) {
        ctx.scratch.rhs = ctx.scratch.admm_neg_c + rho * (ctx.ws.z - ctx.ws.lambda);
        ctx.ws.U = ctx.llt.solve(ctx.scratch.rhs);

        ctx.ws.z = (ctx.ws.U + ctx.ws.lambda)
                       .cwiseMin(ctx.scratch.du_max)
                       .cwiseMax(ctx.scratch.du_min);

        ctx.ws.lambda += ctx.ws.U - ctx.ws.z;
    }

    // ---- 安全弁 1: 双対変数 lambda の暴走を止める ----
    // box が有効になると lambda は毎周期積み上がる。rho が小さすぎると主変数が追いつけず
    // 1 周期で 10-50 倍に発散し、数周期で float を溢れて NaN になる（2026-09-09 実測）。
    // 条件を「< 上限」の否定で書くのは NaN も同時に捕まえるため（NaN は全比較が false）。
    if (!(ctx.ws.lambda.squaredNorm() < rti::admm_lambda_max * rti::admm_lambda_max)) {
        ctx.ws.lambda.setZero();
        ctx.diag.cnt_admm_lambda_reset++;
    }

    // ---- 安全弁 2: 解が非有限になったら warm-start ごと捨てる ----
    // NaN/Inf を後段（duty）まで流すと control.cpp の NaN ガードが minimum_duty=0.1 に
    // 置き換え、スロットルに関係なく全発 0.1 で回り続ける固着になる。
    // ここで捨てて Δu=0（=水平ホバー指令）に落とし、次周期はフル再構築からやり直す。
    if (!ctx.ws.z.allFinite() || !ctx.ws.U.allFinite() || !ctx.ws.lambda.allFinite()) {
        warm_start_reset_to_ref(ctx, U_ref);
        ctx.ws.valid = false;
        ctx.need_full_rebuild = true;
        ctx.rebuild_cause_pending = NMPC_REBUILD_SR2_UNHEALTHY;
        ctx.diag.cnt_nonfinite++;
    }

    // ---- 2026-09-19: ここまでの解を「実際に出す値」として確定させる ----
    // 安全弁 1(lambda) と 2(非有限) は出力にも効かせる。安全弁 3 は効かせない。
    ctx.scratch.z_applied = ctx.ws.z;

    // ---- 安全弁 3: 線形化点 |U_lin| の暴走を止める ----
    // ws.z は次周期の U_lin（線形化点）になる。ここが育つと Su/Hessian が現実から離れ、
    // box が有効化 → lambda 成長 → duty が両端に張り付く、という順で必ず落ちる。
    // 0911_v12 はこの経路で 8 周期かけて発散し、2.5 deg のピッチ誤差にフルの機首上げを出した。
    // 単発のスパイクは健全便でも起きる（v10 で 3 周期連続）ので、**連続回数で見る**。
    // 条件を「< 上限」の否定で書くのは NaN も拾うため。
    //
    // ---- 2026-09-15: 判定を **U_ref からの乖離** に変えた ----
    // 旧実装は |z| を原点基準で測っていた。健全時は U_ref = 0 なので原点が正しい参照だが、
    // **耐故障時は U_ref ≠ 0 で、最適解そのものが原点から大きく離れる。**
    //   u_ref_shift = [-u_h, +u_h/2 ×4, -u_h] を N 段積むので
    //   |U_ref| = u_h·sqrt(N·(2·1² + 4·0.5²)) = sqrt(42)·u_h = 6.48·u_h
    // 実測の回帰係数は 6.256（0915_100Hz_v1, n=378, R=0.87）で理論と一致した。
    // → ulin_max = 1.0 は u_hover > 0.16 で必ず超える。313 g の 4 発ホバー u_hover = 0.558 では
    //   |U_lin| = 3.49 = **閾値の 3.5 倍**。つまり離陸できる推力では 100% 誤発火する。
    // 実機 0915_100Hz_v1 では ulin_reset=198、50 ms ごとに warm start 破棄 → 39 ms のフル再構築、
    // その間 ZOH で「健全 5 発すべて u_hover」（ヨートルク +u_hover ＝ +194 deg/s²）を
    // **飛行時間の 50%** 出し続け、ヨーが +221 deg/s まで発散して墜落した。
    //
    // 健全時は U_ref = 0 なので式は旧実装と**完全に一致**する。v12 墜落の検出能力
    //（t=19.78 発火、q の跳ねの 0.08 s 前）はそのまま保たれる。
    // 根拠: docs/ai/decisions/20260915_ulin_guard_misfires_under_ftc.md
    //
    // ---- 2026-09-19: (a) しきい値を box 相対に (b) 出力は捨てない ----
    // ws.z は box へ射影済みなので |ws.z| の上限は sqrt(NU_TOTAL)*half_span で決まる。
    // 固定 1.0 はその 27% しかなく、正常な大操作で発火していた。
    // 発火してもこの周期の duty は z_applied（上でコピー済み）を使うので影響しない。
    // 根拠: docs/ai/decisions/20260919_payload_drop_crash_ulin_guard_zeroes_du.md
    const float ulin_half_span =
        (u_hover > (mpc::U_duty_max - u_hover)) ? u_hover : (mpc::U_duty_max - u_hover);
    float ulin_lim = rti::ulin_box_frac * sqrtf((float)mpc::NU_TOTAL) * ulin_half_span;
    if (ulin_lim < rti::ulin_min_abs) ulin_lim = rti::ulin_min_abs;
    if (!((ctx.ws.z - U_ref).squaredNorm() < ulin_lim * ulin_lim)) {
        if (++ctx.ulin_hot_run >= rti::ulin_max_run) {
            ctx.ulin_hot_run = 0;
            // ---- 2026-09-15: フル再構築を**要求しない**（案 A）----
            // このガードが止めたいのは線形化点の暴走であって、Hessian は壊れていない。
            // 旧実装は need_full_rebuild を立てていたため、発火のたびに 39 ms のフル再構築が入り、
            // 100 Hz ループが 3 ティック落として **その間 ZOH で古い duty を保持**していた。
            // 0915_100Hz_v1 では 50 ms ごとに発火したので、飛行時間の 50% がこのストールだった。
            // warm start を U_ref へ戻せば、次周期は通常の TR1/SR2 パス（9.5 ms）で
            // U_ref 周りに再線形化される。
            // ⚠ TR1 はランク 1 更新なので z → U_ref の大ジャンプを 1 回で捉えきれないことがある。
            //   その場合は secant ガードが TR1 を skip し、SR2 の健全性チェックが
            //   本当に壊れたときだけフル再構築を要求する（degradation は graceful）。
            // ⚠ 非有限（安全弁 2）と LLT 不健全は今まで通りフル再構築を要求する。
            //   あちらは Hessian そのものが使えないので 39 ms を払う価値がある。
            warm_start_reset_to_ref(ctx, U_ref);
            ctx.ws.valid = false;
            ctx.diag.cnt_ulin_reset++;
        }
    } else {
        ctx.ulin_hot_run = 0;
    }
    return iters;
}

void warm_start_shift(WarmStart &ws)
{
    if (!ws.valid) return;
    auto shift = [](UStackVec &v) {
        for (int k = 0; k < mpc::N - 1; ++k) {
            for (int j = 0; j < mpc::NU; ++j) {
                v(k * mpc::NU + j) = v((k + 1) * mpc::NU + j);
            }
        }
    };
    shift(ws.U);
    shift(ws.z);
    shift(ws.lambda);
}

void do_full_rebuild(NmpcContext &ctx,
                     const StateVec &x0,
                     const UStackVec &U_lin,
                     uint8_t cause,
                     uint8_t path)
{
    full_rebuild_sx_su(ctx, x0, U_lin);
    full_rebuild_chol(ctx);
    ctx.cycles_since_reset = 0;
    ctx.tr1_skip_run = 0;
    ctx.ulin_hot_run = 0;
    ctx.need_full_rebuild = false;
    ctx.rebuild_cause_pending = NMPC_REBUILD_NONE;
    ctx.last_was_reset = true;
    ctx.diag.last_reset_cause = cause;
    ctx.diag.last_path = path;
}

}  // namespace

void nmpc_set_motor_eta(NmpcContext &ctx, const float eta[mpc::NU])
{
    for (int j = 0; j < mpc::NU; ++j) {
        float e = eta[j];
        if (!(e >= 0.0f)) e = 0.0f;   // NaN も 0 に倒す
        if (e > 1.0f) e = 1.0f;
        ctx.motor_eta[j] = e;
    }
    // Hessian と LLT は box に依存しないのでフル再構築は不要。
    // warm-start も、次周期の射影で新しい box に落ちるためそのまま使える。
}

bool nmpc_delay_predict(const StateVec &x0, const InputVec *hist, int n, float dt, StateVec &x_pred)
{
    // 1 周期を 4 分割した中点法（RK2）。model_f は連続時間 dx/dt = f(x,u)、u は絶対 duty。
    // 記録状態リプレイで、墜落域（角速度 1000 deg/s 超・duty 全開）の状態を 1 周期刻みで積分すると
    // 二乗減衰項が硬くなり 1e9 deg まで発散した（2026-09-10）。刻みを細かくし、さらに下の健全性判定で弾く。
    constexpr int   SUB = 4;
    const float     h   = dt / SUB;
    StateVec x = x0, k1, k2, xm;
    for (int i = 0; i < n; ++i) {
        for (int s = 0; s < SUB; ++s) {
            model_f(x, hist[i], k1);
            xm = x + 0.5f * h * k1;
            model_f(xm, hist[i], k2);
            x += h * k2;
        }
    }
    // 健全性: 遅れ時間ぶんの予測変化が物理的にあり得る範囲か。超えたら補償せず x0 を使う。
    //   角度変化 < 20 deg、角速度変化 < 10 rad/s (573 deg/s)。正常飛行では角度数 deg・角速度数十 deg/s。
    const float dphi = x(0) - x0(0), dth = x(1) - x0(1);
    const float dp = x(2) - x0(2), dq = x(3) - x0(3), dr = x(4) - x0(4);
    const bool ok = x.allFinite()
                 && fabsf(dphi) < 0.35f && fabsf(dth) < 0.35f
                 && fabsf(dp) < 10.0f && fabsf(dq) < 10.0f && fabsf(dr) < 10.0f;
    if (!ok) { x_pred = x0; return false; }
    x_pred = x;
    return true;
}

void nmpc_mark_external_rebuild(NmpcContext &ctx)
{
    ctx.need_full_rebuild = true;
    ctx.rebuild_cause_pending = NMPC_REBUILD_EXTERNAL;
    ctx.ws.valid = false;
}

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
    ctx.tr1_skip_run = 0;
    ctx.ulin_hot_run = 0;
    ctx.scratch.U_lin.setZero();
    ctx.scratch.g.setZero();
    ctx.scratch.X_nl.setZero();
    ctx.scratch.rhs.setZero();
    ctx.scratch.admm_neg_c.setZero();
    ctx.scratch.tr1_residual.setZero();
    ctx.scratch.tr1_u_rk1.setZero();
    ctx.scratch.tr1_s_u.setZero();
    ctx.scratch.sr2_Q_u.setZero();
    ctx.scratch.sr2_a.setZero();
    ctx.scratch.sr2_p.setZero();
    ctx.scratch.sr2_q.setZero();
    ctx.scratch.grad_dx.setZero();
    ctx.scratch.grad_Qdx.setZero();
    ctx.scratch.eval_dx.setZero();
    ctx.scratch.eval_du.setZero();
    ctx.scratch.du_min.setZero();
    ctx.scratch.du_max.setZero();
    for (int j = 0; j < mpc::NU; ++j) {
        ctx.motor_eta[j] = 1.0f;   // 既定は全機健全
    }
    ctx.scratch.Su_w.setZero();
    ctx.cycles_since_reset = 0;
    ctx.need_full_rebuild = true;
    ctx.rebuild_cause_pending = NMPC_REBUILD_INIT;
    ctx.last_admm_iters = 0;
    ctx.initialized = true;
    ctx.last_cost = 0.0f;
    ctx.last_was_reset = false;
    ctx.diag = {};
}

void nmpc_warmup(NmpcContext &ctx)
{
    if (!ctx.initialized) {
        nmpc_init(ctx);
    }

    StateVec x0 = StateVec::Zero();
    ctx.scratch.U_lin.setZero();
    do_full_rebuild(ctx, x0, ctx.scratch.U_lin,
                    NMPC_REBUILD_INIT, NMPC_PATH_RESET_START);
    ctx.diag = {};
}

void nmpc_step(NmpcContext &ctx,
               const StateVec &x0,
               const XStackVec &X_ref,
               const UStackVec &U_ref,
               float u_hover,
               InputVec &U_opt,
               float &cost_out)
{
    if (!ctx.initialized) {
        nmpc_init(ctx);
    }

    warm_start_shift(ctx.ws);
    // 線形化点は **射影後の z**（= 実際にモータへ出た Δu）にする（2026-09-10）。
    // ws.U は ADMM の射影前の主変数で box に縛られず、大きな角速度が入った周期には
    // 物理上限 (√120×0.95=10.4) を桁で超えることがある（0910_100Hz_v4 リプレイで 8599）。
    // そこで線形化すると Su/Sx が無意味になり、非有限へ落ちて安全弁 2 が Δu=0 を出す。
    // 機体を駆動しているのは z なので、軌道の線形化も z 周りが正しい。
    // リプレイ 4 飛行で |U_lin|max 8599→4.25、nonfinite 1→0、λ リセット 7→3、実機 duty との差は同等。
    // 静的ゲイン・閉ループ応答は不変（ホスト確認）。
#if NMPC_LEGACY_SOLVER
    // 旧: 射影前の主変数で線形化。旧写像 U_new = -U_lin - H^{-1}c の固有値は -1 で、
    // Su の零空間（姿勢にも推力にも効かない方向）の成分は減衰も励起もされず符号だけ反転し続ける。
    // 雑音で積み上がった零空間成分は box に当たるまで成長する（0910_v9: 1 周期跳びの 65-70% が
    // 零空間成分で、0.15 → 0.81 と単調増加、13.5 s に box 有効 → λ 成長 → ロール権限喪失 → 墜落。
    // 0909_v1 の 31 s 以降も同じ）。legacy_gamma < 1 で固有値を -γ に下げ、この成長を止める。
    // 不動点は -H^{-1}c/(1+γ)（γ=1 で解の 1/2、γ=0.7 で 0.59）。
    // ⚠ γ は compute_neg_gradient_legacy() の中でだけ掛ける。線形化点（rollout/TR1/SR2）まで
    //   γ·ws.U にすると割線条件が崩れ、リプレイで |U_lin| が 1e4 まで発散した（2026-09-10 ホスト）。
    ctx.scratch.U_lin = ctx.ws.U;
#else
    ctx.scratch.U_lin = ctx.ws.z;
#endif
    UStackVec &U_lin = ctx.scratch.U_lin;

    const bool had_pending = ctx.need_full_rebuild;
    // 周期フル再構築は 58 ms 級のストールを起こすため、離陸し得ない低スロットル域でのみ許可する。
    // 飛行中は TR1/SR2 の差分更新に任せ、異常時は SR2 健全性チェック側で拾う。
    const bool rebuild_allowed = (u_hover < rti::rebuild_max_u_hover);
    const bool period_due = rebuild_allowed && (ctx.cycles_since_reset >= rti::reset_period);
    const bool reset_now = had_pending || period_due;

    ctx.diag.last_tr1_applied = 0;

    if (reset_now) {
        uint8_t cause = NMPC_REBUILD_PERIOD;
        if (had_pending) {
            cause = (ctx.rebuild_cause_pending != NMPC_REBUILD_NONE)
                        ? ctx.rebuild_cause_pending
                        : NMPC_REBUILD_EXTERNAL;
            bump_rebuild_counter(ctx.diag, cause);
        }
        if (period_due) {
            if (!had_pending) {
                cause = NMPC_REBUILD_PERIOD;
            }
            bump_rebuild_counter(ctx.diag, NMPC_REBUILD_PERIOD);
        }

        do_full_rebuild(ctx, x0, U_lin, cause, NMPC_PATH_RESET_START);
    } else {
        ctx.diag.last_path = NMPC_PATH_TR1_SR2;
        ctx.diag.last_reset_cause = NMPC_REBUILD_NONE;
        ctx.diag.cnt_tr1_path++;

        nonlinear_rollout(x0, U_lin, ctx.scratch.X_nl);

        const bool tr1_applied = tr1_update_sx_su(ctx, x0, U_lin, ctx.scratch.X_nl);
        ctx.diag.last_tr1_applied = tr1_applied ? 1u : 0u;
        if (tr1_applied) {
            ctx.diag.cnt_tr1_applied++;
        } else {
            ctx.diag.cnt_tr1_skipped++;
        }

        sr2_update_h(ctx, tr1_applied);

        ctx.cycles_since_reset += 1;
        ctx.last_was_reset = false;

        if (ctx.need_full_rebuild) {
            const uint8_t cause = (ctx.rebuild_cause_pending != NMPC_REBUILD_NONE)
                                      ? ctx.rebuild_cause_pending
                                      : NMPC_REBUILD_SR2_UNHEALTHY;
            ctx.diag.cnt_reset_after_sr2++;
            bump_rebuild_counter(ctx.diag, cause);
            do_full_rebuild(ctx, x0, U_lin, cause, NMPC_PATH_RESET_AFTER_SR2);
        }
    }

    ctx.diag.cycles_since_reset = ctx.cycles_since_reset;

#if NMPC_LEGACY_SOLVER
    compute_neg_gradient_legacy(ctx, x0, X_ref, U_ref, U_lin);
#else
    compute_neg_const_term(ctx, x0, X_ref, U_ref);
#endif
    ctx.last_admm_iters = admm_solve(ctx, u_hover, U_ref);

    // admm_solve が張り直した box をそのまま使う（モータごと）
    for (int j = 0; j < mpc::NU; ++j) {
        const float lo = ctx.scratch.du_min(j);
        const float hi = ctx.scratch.du_max(j);
        // ⚠ ws.z ではなく z_applied。安全弁 3 が ws.z を U_ref へ戻していても、
        //    この周期の制御出力は捨てない（2026-09-19）。
        float v = ctx.scratch.z_applied(j);
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        U_opt(j) = v;
    }

#if NMPC_BENCH_COST
    cost_out = evaluate_cost(ctx, x0, X_ref, U_ref, ctx.scratch.z_applied);
    ctx.last_cost = cost_out;
#else
    cost_out = 0.0f;
    ctx.last_cost = 0.0f;
#endif

    ctx.x0_prev = x0;
    ctx.U_lin_prev = U_lin;
    linear_predict(ctx, x0, U_lin, ctx.X_lin_prev);
}

NmpcCostParams nmpc_cost_defaults()
{
    NmpcCostParams p;
    p.Q_phi = cost::Q_phi;
    p.Q_theta = cost::Q_theta;
    p.Q_p = cost::Q_p;
    p.Q_q = cost::Q_q;
    p.Q_r = cost::Q_r;
    p.R_u = cost::R_u;
    p.terminal_scale = cost::terminal_scale;
    return p;
}

void nmpc_cost_init()
{
    if (!g_cost_inited) {
        g_cost_base = nmpc_cost_defaults();
        g_cost_active = g_cost_base;
        build_cost_diag(g_cost_diag, g_cost_active);
        g_cost_inited = true;
    }
}

NmpcCostParams nmpc_cost_get_base()
{
    cost_diag_ref();
    return g_cost_base;
}

NmpcCostParams nmpc_cost_get_active()
{
    cost_diag_ref();
    return g_cost_active;
}

float nmpc_schedule_alpha(float u_hover)
{
    (void)u_hover;
    return 0.0f;
}

float nmpc_schedule_alpha_last(void)
{
    return 0.0f;
}

NmpcCostParams nmpc_cost_schedule(float u_hover, const NmpcCostParams &base)
{
    (void)u_hover;
    return base;
}

void nmpc_cost_set_pending(const NmpcCostParams &p)
{
    g_cost_pending = clamp_cost_params(p);
    g_cost_pending_flag = 1;
}

bool nmpc_cost_apply_pending(NmpcContext &ctx)
{
    if (!g_cost_pending_flag)
        return false;
    g_cost_pending_flag = 0;
    g_cost_base = g_cost_pending;
    g_cost_active = g_cost_pending;
    build_cost_diag(g_cost_diag, g_cost_active);
    g_cost_inited = true;
    nmpc_mark_external_rebuild(ctx);
    return true;
}

bool nmpc_cost_update_scheduled(float u_hover, NmpcContext &ctx)
{
    (void)u_hover;
    (void)ctx;
    return false;
}

}  // namespace nmpc
