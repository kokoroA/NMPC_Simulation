// nmpc_model.cpp ----------------------------------------------------------
// 4 入力姿勢 NMPC のモデル実装
//
// 解析微分は gen_jacobi_4input.py の出力と等価だが、
//   |p| → sqrt(p^2 + eps^2)
// による平滑化を施した版を直書きしている。
// Python 側のヘッダは検算/参照用の位置づけ。
// -------------------------------------------------------------------------
#include "nmpc_model.hpp"

#include <math.h>

namespace nmpc {

namespace {

inline float smooth_abs(float v)
{
    const float e = mpc::damping_eps;
    return sqrtf(v * v + e * e);
}

inline float smooth_abs_d(float v)
{
    const float e = mpc::damping_eps;
    return v / sqrtf(v * v + e * e);
}

}  // namespace

void model_f(const StateVec &x, const InputVec &u, StateVec &xdot)
{
    using namespace physical;

    const float phi   = x(0);
    const float theta = x(1);
    const float p     = x(2);
    const float q     = x(3);
    const float r     = x(4);

    const float u_fr = u(0);
    const float u_fl = u(1);
    const float u_rr = u(2);
    const float u_rl = u(3);

    const float tau_phi   = 0.5f * arm_length * T_max * (u_fl + u_rl - u_fr - u_rr);
    const float tau_theta = 0.5f * arm_length * T_max * (u_fr + u_fl - u_rr - u_rl);
    const float tau_r     =        C_QT       * T_max * (u_fr + u_rl - u_fl - u_rr);

    const float ap = smooth_abs(p);
    const float aq = smooth_abs(q);
    const float ar = smooth_abs(r);

    const float sphi   = sinf(phi);
    const float cphi   = cosf(phi);
    const float ttheta = tanf(theta);

    xdot(0) = p + q * sphi * ttheta + r * cphi * ttheta;
    xdot(1) = q * cphi - r * sphi;
    xdot(2) = (tau_phi   - (Iz - Iy) * q * r - Cp * p * ap) / Ix;
    xdot(3) = (tau_theta - (Ix - Iz) * r * p - Cq * q * aq) / Iy;
    xdot(4) = (tau_r     - (Iy - Ix) * p * q - Cr * r * ar) / Iz;
}

void model_B_const(InputMat &B_c)
{
    using namespace physical;
    B_c.setZero();

    const float kp = 0.5f * arm_length * T_max;
    const float kr = C_QT * T_max;

    B_c(2, 0) = -kp / Ix;
    B_c(2, 1) =  kp / Ix;
    B_c(2, 2) = -kp / Ix;
    B_c(2, 3) =  kp / Ix;

    B_c(3, 0) =  kp / Iy;
    B_c(3, 1) =  kp / Iy;
    B_c(3, 2) = -kp / Iy;
    B_c(3, 3) = -kp / Iy;

    B_c(4, 0) =  kr / Iz;
    B_c(4, 1) = -kr / Iz;
    B_c(4, 2) = -kr / Iz;
    B_c(4, 3) =  kr / Iz;
}

void model_jacobians(const StateVec &x, const InputVec &u,
                     StateMat &A_c, InputMat &B_c)
{
    using namespace physical;
    (void)u;

    const float phi   = x(0);
    const float theta = x(1);
    const float p     = x(2);
    const float q     = x(3);
    const float r     = x(4);

    const float sphi = sinf(phi);
    const float cphi = cosf(phi);
    float ctheta     = cosf(theta);
    if (fabsf(ctheta) < 1.0e-3f) {
        ctheta = (ctheta < 0.0f) ? -1.0e-3f : 1.0e-3f;
    }
    const float ttheta = sinf(theta) / ctheta;
    const float sec2   = 1.0f / (ctheta * ctheta);

    A_c.setZero();

    A_c(0, 0) = (q * cphi - r * sphi) * ttheta;
    A_c(0, 1) = (q * sphi + r * cphi) * sec2;
    A_c(0, 2) = 1.0f;
    A_c(0, 3) = sphi * ttheta;
    A_c(0, 4) = cphi * ttheta;

    A_c(1, 0) = -q * sphi - r * cphi;
    A_c(1, 3) =  cphi;
    A_c(1, 4) = -sphi;

    {
        const float ap  = smooth_abs(p);
        const float dap = smooth_abs_d(p);
        A_c(2, 2) = -Cp * (ap + p * dap) / Ix;
        A_c(2, 3) = -(Iz - Iy) * r / Ix;
        A_c(2, 4) = -(Iz - Iy) * q / Ix;
    }
    {
        const float aq  = smooth_abs(q);
        const float daq = smooth_abs_d(q);
        A_c(3, 2) = -(Ix - Iz) * r / Iy;
        A_c(3, 3) = -Cq * (aq + q * daq) / Iy;
        A_c(3, 4) = -(Ix - Iz) * p / Iy;
    }
    {
        const float ar  = smooth_abs(r);
        const float dar = smooth_abs_d(r);
        A_c(4, 2) = -(Iy - Ix) * q / Iz;
        A_c(4, 3) = -(Iy - Ix) * p / Iz;
        A_c(4, 4) = -Cr * (ar + r * dar) / Iz;
    }

    model_B_const(B_c);
}

void discretize_euler(const StateMat &A_c, const InputMat &B_c,
                      StateMat &A_d, InputMat &B_d)
{
    A_d = StateMat::Identity() + mpc::dt * A_c;
    B_d = mpc::dt * B_c;
}

void simulate_step(const StateVec &x, const InputVec &u, StateVec &x_next)
{
    StateVec xdot;
    model_f(x, u, xdot);
    x_next = x + mpc::dt * xdot;
}

}  // namespace nmpc
