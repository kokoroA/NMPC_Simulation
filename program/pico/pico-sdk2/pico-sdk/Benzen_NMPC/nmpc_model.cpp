// nmpc_model.cpp ----------------------------------------------------------
// 6 入力姿勢 NMPC のモデル実装 (Hexa-X)
//
// nmpc_simulation.py の HexaModel と 1:1 対応。
//   xdot[2:5] = B_c * u + (慣性 + 空力ダンピング)
//   |p| → sqrt(p^2 + eps^2) で平滑化。
//
// モータ配置 (実機順序 [FR, FL, MR, ML, RR, RL])
//   正六角形フレームの各頂点に配置。座標は重心からの実測値 [m]
//   (x: 前+, y: 右+), s_real: ヨー反トルク符号 (実機ミキサ R 係数の符号)
//     FR (+0.085, +0.050, -1)   FL (+0.085, -0.050, +1)
//     MR ( 0.0  , +0.100, +1)   ML ( 0.0  , -0.100, -1)
//     RR (-0.085, +0.050, -1)   RL (-0.085, -0.050, +1)
//   B_c[2,k] = -y_k T_max / Ix   (roll)
//   B_c[3,k] = +x_k T_max / Iy   (pitch)
//   B_c[4,k] = s_k C_QT T_max / Iz (yaw)
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

// 実機モータ位置 [m] (FR, FL, MR, ML, RR, RL)
constexpr float kXm[mpc::NU] = {  0.085f,  0.085f,  0.0f,   0.0f,  -0.085f, -0.085f };
constexpr float kYm[mpc::NU] = {  0.050f, -0.050f,  0.10f, -0.10f,   0.050f, -0.050f };
constexpr float kS[mpc::NU]  = { -1.0f,   1.0f,   1.0f, -1.0f,  -1.0f,    1.0f  };

}  // namespace

void model_B_const(InputMat &B_c)
{
    using namespace physical;
    B_c.setZero();

    for (int k = 0; k < mpc::NU; ++k) {
        B_c(2, k) = -kYm[k] * T_max / Ix;  // roll
        B_c(3, k) =  kXm[k] * T_max / Iy;  // pitch
        B_c(4, k) =  kS[k] * C_QT       * T_max / Iz;  // yaw
    }
}

void model_f(const StateVec &x, const InputVec &u, StateVec &xdot)
{
    using namespace physical;

    const float phi   = x(0);
    const float theta = x(1);
    const float p     = x(2);
    const float q     = x(3);
    const float r     = x(4);

    InputMat B_c;
    model_B_const(B_c);
    const StateVec Bu = B_c * u;  // 非ゼロは行 2,3,4 (既に Ix/Iy/Iz で割済)

    const float ap = smooth_abs(p);
    const float aq = smooth_abs(q);
    const float ar = smooth_abs(r);

    const float sphi   = sinf(phi);
    const float cphi   = cosf(phi);
    const float ttheta = tanf(theta);

    xdot(0) = p + q * sphi * ttheta + r * cphi * ttheta;
    xdot(1) = q * cphi - r * sphi;
    xdot(2) = Bu(2) + (-(Iz - Iy) * q * r - Cp * p * ap) / Ix;
    xdot(3) = Bu(3) + (-(Ix - Iz) * r * p - Cq * q * aq) / Iy;
    xdot(4) = Bu(4) + (-(Iy - Ix) * p * q - Cr * r * ar) / Iz;
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

bool model_sanity(void)
{
    InputMat B_c;
    model_B_const(B_c);

    // 1) B_c * 1 = 0 (全6発均等で姿勢トルク 0)
    InputVec ones = InputVec::Ones();
    StateVec Bsum = B_c * ones;
    for (int i = 2; i < mpc::NX; ++i) {
        if (fabsf(Bsum(i)) > 1.0e-3f) return false;
    }

    // 2) 姿勢行 B_c[2:5,:] のランク = 3
    Eigen::Matrix<float, 3, mpc::NU> Batt = B_c.bottomRows<3>();
    Eigen::FullPivLU<Eigen::Matrix<float, 3, mpc::NU>> lu(Batt);
    if (lu.rank() != 3) return false;

    return true;
}

}  // namespace nmpc
