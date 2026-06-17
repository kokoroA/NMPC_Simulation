// nmpc_model.hpp ----------------------------------------------------------
// 姿勢制御 NMPC のプラントモデル (4 入力版 / フェーズ2)
//
//   状態 x = [phi, theta, p, q, r]^T   (5 次元)
//   入力 u = [u_fr, u_fl, u_rr, u_rl]^T (4 次元 / 各モータ正規化推力 0..1)
//
// 提供 API
//   - model_f          : 連続時間ダイナミクス  dx/dt = f(x,u)
//   - model_jacobians  : 解析ヤコビアン A_c = ∂f/∂x, B_c = ∂f/∂u
//                        |p| を sqrt(p^2 + eps^2) で平滑近似した式を採用
//   - discretize_euler : Euler 離散化  A_d = I + dt A_c, B_d = dt B_c
//   - simulate_step    : x_{k+1} = x_k + dt f(x_k, u_k)  (非線形 1 ステップ)
//   - model_B_const    : (x,u) に依存しない B_c だけを返す高速版
// -------------------------------------------------------------------------
#ifndef NMPC_MODEL_HPP
#define NMPC_MODEL_HPP

#include <Eigen/Dense>
#include "nmpc_params.hpp"

namespace nmpc {

using StateVec = Eigen::Matrix<float, mpc::NX, 1>;
using InputVec = Eigen::Matrix<float, mpc::NU, 1>;
using StateMat = Eigen::Matrix<float, mpc::NX, mpc::NX>;
using InputMat = Eigen::Matrix<float, mpc::NX, mpc::NU>;

void model_f(const StateVec &x, const InputVec &u, StateVec &xdot);

void model_jacobians(const StateVec &x, const InputVec &u,
                     StateMat &A_c, InputMat &B_c);

void model_B_const(InputMat &B_c);

void discretize_euler(const StateMat &A_c, const InputMat &B_c,
                      StateMat &A_d, InputMat &B_d);

void simulate_step(const StateVec &x, const InputVec &u, StateVec &x_next);

}  // namespace nmpc

#endif  // NMPC_MODEL_HPP
