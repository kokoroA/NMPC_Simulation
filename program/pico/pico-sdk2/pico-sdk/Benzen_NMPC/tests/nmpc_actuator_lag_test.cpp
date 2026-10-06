#if !defined(EIGEN_NO_MALLOC) && !defined(EIGEN_RUNTIME_NO_MALLOC)
#define EIGEN_RUNTIME_NO_MALLOC
#endif

#include <Eigen/Dense>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#if defined(__APPLE__) || defined(__linux__)
#include <csignal>
#include <execinfo.h>
#include <unistd.h>
#endif

#include "../nmpc.hpp"
#include "../nmpc_model.hpp"

namespace {

int failures = 0;

void check(bool ok, const char *name)
{
    std::printf("%s: %s\n", name, ok ? "PASS" : "FAIL");
    if (!ok) ++failures;
}

bool close_mixed(float actual, float expected, float atol = 1.0e-5f, float rtol = 1.0e-3f)
{
    return std::fabs(actual - expected) <= atol + rtol * std::fabs(expected);
}

void test_reduced_equivalence()
{
    nmpc::AttInputMat B;
    nmpc::model_B_att(B);
    uint32_t rng = 0x12345678u;
    bool ok = true;
    for (float tau : {0.035f, 0.045f, 0.060f}) {
        const float alpha = std::exp(-nmpc::mpc::dt / tau);
        nmpc::InputVec omega = nmpc::InputVec::Zero();
        nmpc::AttAccelVec a3 = B * omega;
        for (int k = 0; k < 1000; ++k) {
            nmpc::InputVec u;
            for (int j = 0; j < nmpc::mpc::NU; ++j) {
                rng = 1664525u * rng + 1013904223u;
                u(j) = static_cast<float>((rng >> 8) & 0xffffu) / 65535.0f;
            }
            omega = alpha * omega + (1.0f - alpha) * u;
            a3 = alpha * a3 + (1.0f - alpha) * (B * u);
            const float err = (B * omega - a3).cwiseAbs().maxCoeff();
            if (err > 1.0e-4f) ok = false;
        }
    }
    check(ok, "T3 six-motor/reduced equivalence");
}

void test_step_response()
{
    nmpc::AttInputMat B;
    nmpc::model_B_att(B);
    nmpc::InputVec u;
    u << 0.8f, 0.2f, 0.7f, 0.3f, 0.6f, 0.4f;
    const nmpc::AttAccelVec target = B * u;
    nmpc::AttAccelVec a = nmpc::AttAccelVec::Zero();
    const int steps = 450;  // 45 ms at 0.1 ms
    const float dt = nmpc::physical::tau_act / static_cast<float>(steps);
    const float alpha = std::exp(-dt / nmpc::physical::tau_act);
    for (int i = 0; i < steps; ++i) a = alpha * a + (1.0f - alpha) * target;
    check((a - (1.0f - std::exp(-1.0f)) * target).cwiseAbs().maxCoeff() < 1.0e-3f,
          "T4 63.2 percent response");
}

void test_jacobian_case(const nmpc::StateVec &x, const nmpc::InputVec &u, const char *name)
{
    nmpc::StateMat Ac, Ad;
    nmpc::InputMat Bc, Bd;
    nmpc::model_jacobians(x, u, Ac, Bc);
    nmpc::discretize_euler(Ac, Bc, Ad, Bd);
    bool ok = true;
    float worst_abs = 0.0f;
    float worst_rel = 0.0f;

    for (int j = 0; j < nmpc::mpc::NX; ++j) {
        const float h = (j < 2) ? 5.0e-4f : ((j < 5) ? 2.0e-3f : 1.0e-2f);
        nmpc::StateVec xp = x, xm = x, fp, fm;
        xp(j) += h;
        xm(j) -= h;
        nmpc::simulate_step(xp, u, fp);
        nmpc::simulate_step(xm, u, fm);
        const nmpc::StateVec fd = (fp - fm) / (2.0f * h);
        for (int i = 0; i < nmpc::mpc::NX; ++i) {
            const float ae = std::fabs(fd(i) - Ad(i, j));
            const float re = ae / (std::fabs(Ad(i, j)) + 1.0e-6f);
            if (ae > worst_abs) worst_abs = ae;
            if (re > worst_rel && std::fabs(Ad(i, j)) > 1.0e-4f) worst_rel = re;
            if (!close_mixed(fd(i), Ad(i, j), 3.0e-4f, 1.0e-3f)) ok = false;
        }
    }
    for (int j = 0; j < nmpc::mpc::NU; ++j) {
        const float h = 5.0e-3f;
        nmpc::InputVec up = u, um = u;
        nmpc::StateVec fp, fm;
        up(j) += h;
        um(j) -= h;
        nmpc::simulate_step(x, up, fp);
        nmpc::simulate_step(x, um, fm);
        const nmpc::StateVec fd = (fp - fm) / (2.0f * h);
        for (int i = 0; i < nmpc::mpc::NX; ++i) {
            const float ae = std::fabs(fd(i) - Bd(i, j));
            const float re = ae / (std::fabs(Bd(i, j)) + 1.0e-6f);
            if (ae > worst_abs) worst_abs = ae;
            if (re > worst_rel && std::fabs(Bd(i, j)) > 1.0e-4f) worst_rel = re;
            if (!close_mixed(fd(i), Bd(i, j), 3.0e-4f, 1.0e-3f)) ok = false;
        }
    }
    std::printf("  worst_abs=%.3e worst_rel=%.3e\n", worst_abs, worst_rel);
    check(ok, name);
}

void test_failure_decay()
{
    nmpc::AttInputMat B;
    nmpc::model_B_att(B);
    nmpc::InputVec omega;
    omega << 0.8f, 0.3f, 0.7f, 0.4f, 0.6f, 0.5f;
    nmpc::AttAccelVec a3 = B * omega;
    nmpc::InputVec u = omega;
    u(0) = 0.0f;
    const float alpha = std::exp(-nmpc::mpc::dt / nmpc::physical::tau_act);
    bool ok = true;
    for (int k = 0; k < 100; ++k) {
        omega = alpha * omega + (1.0f - alpha) * u;
        a3 = alpha * a3 + (1.0f - alpha) * (B * u);
        if ((B * omega - a3).cwiseAbs().maxCoeff() > 1.0e-4f) ok = false;
    }
    check(ok, "T6 failed-motor residual decay");
}

void test_solver_static()
{
    static nmpc::NmpcContext ctx;
    nmpc::nmpc_cost_init();
    nmpc::nmpc_init(ctx);
    nmpc::nmpc_warmup(ctx);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(false);
#endif
    nmpc::StateVec x0 = nmpc::StateVec::Zero();
    x0(0) = 5.0f * 3.14159265f / 180.0f;
    nmpc::XStackVec xr = nmpc::XStackVec::Zero();
    nmpc::UStackVec ur = nmpc::UStackVec::Zero();
    nmpc::InputVec uopt;
    float cost = 0.0f;
    nmpc::nmpc_step(ctx, x0, xr, ur, 0.55f, uopt, cost);
#ifdef EIGEN_RUNTIME_NO_MALLOC
    Eigen::internal::set_is_malloc_allowed(true);
#endif
    const bool finite = uopt.allFinite() && ctx.M.allFinite()
                     && ctx.llt.info() == Eigen::Success;
    bool in_box = true;
    for (int j = 0; j < nmpc::mpc::NU; ++j)
        in_box = in_box && uopt(j) >= -0.55f - 1.0e-6f
                        && uopt(j) <= nmpc::mpc::U_duty_max - 0.55f + 1.0e-6f;
    check(finite && in_box && ctx.last_admm_iters == nmpc::rti::admm_max_iter,
          "solver finite/SPD/ADMM/box/no-malloc");
}

}  // namespace

int main()
{
#if defined(__APPLE__) || defined(__linux__)
    std::signal(SIGABRT, [](int) {
        void *frames[32];
        const int n = backtrace(frames, 32);
        backtrace_symbols_fd(frames, n, STDERR_FILENO);
        _exit(134);
    });
#endif
    nmpc::AttInputMat B;
    nmpc::model_B_att(B);
    check((B * nmpc::InputVec::Ones()).cwiseAbs().maxCoeff() <= 1.0e-5f,
          "T1 common mode");
    check(Eigen::FullPivLU<nmpc::AttInputMat>(B).rank() == 3 && nmpc::model_sanity(),
          "T2 rank/model sanity");
    test_reduced_equivalence();
    test_step_response();

    nmpc::InputVec u = nmpc::InputVec::Constant(0.5f);
    nmpc::StateVec x = nmpc::StateVec::Zero();
    x << 0.12f, -0.09f, 0.4f, -0.3f, 0.2f, 12.0f, -8.0f, 3.0f;
    test_jacobian_case(x, u, "T5 Jacobian nominal");
    x(2) = 300.0f * 3.14159265f / 180.0f;
    x(3) = -x(2);
    x(4) = x(2);
    u.setConstant(0.94f);
    test_jacobian_case(x, u, "T5 Jacobian high-rate/near-box");
    test_failure_decay();
    test_solver_static();
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
