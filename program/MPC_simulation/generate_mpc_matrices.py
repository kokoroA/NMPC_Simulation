#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np

# ========= パラメータ =========
NX = 5   # nx
NU = 3   # nu
N  = 20  # horizon length

DT_A = 0.01   # A の離散化刻み（C++と同じ）
DT_B = 0.01     # B のスケーリング（現状コードで 0.01 にしている）

RHO_ADMM = 0.1  # C++側の rho_admm と揃える

# ========= C++側と同じ行列の構築 =========

def build_system_matrices():
    """C++側の mpc_init と同じ A, B, Q, R, Qf を作る"""
    # A 行列
    A = np.array([
        [1.0, 0.0,    DT_A, 0.0,    0.0],
        [0.0, 1.0,    0.0,  DT_A,   0.0],
        [0.0, 0.0,    1.0,  0.0,    0.0],
        [0.0, 0.0,    0.0,  1.0,    0.0],
        [0.0, 0.0,    0.0,  0.0,    1.0],
    ], dtype=np.float32)

    # B 行列（あなたの現状コードの version）
    B = np.array([
        [0.0,              0.0,              0.0],
        [0.0,              0.0,              0.0],
        [276.9 * DT_B,     0.0,              0.0],
        [0.0,              276.9 * DT_B,     0.0],
        [0.0,              0.0,              304.6 * DT_B],
    ], dtype=np.float32)

    # 重み行列 Q
    Q = np.eye(NX, dtype=np.float32)
    Q[0, 0] = 20.0
    Q[1, 1] = 20.0
    Q[2, 2] = 0.1
    Q[3, 3] = 0.1
    Q[4, 4] = 20.0

    q_c = 0.1
    Q[0, 2] = q_c
    Q[2, 0] = q_c
    Q[1, 3] = q_c
    Q[3, 1] = q_c

    # 重み行列 R
    R = np.eye(NU, dtype=np.float32)
    R[0, 0] = 1.0
    R[1, 1] = 1.0
    R[2, 2] = 1.0

    # 終端重み Qf
    Qf = Q.copy()
    Qf[0, 0] *= 30.0
    Qf[1, 1] *= 30.0

    return A, B, Q, R, Qf


def build_prediction_matrices(A, B):
    """A_aug, B_aug を C++ と同じロジックで構築"""
    nx, nu = A.shape[0], B.shape[1]
    nxN = nx * N
    nuN = nu * N

    A_aug = np.zeros((nxN, nx), dtype=np.float32)
    B_aug = np.zeros((nxN, nuN), dtype=np.float32)

    # A^{i+1}
    Apow = A.copy()
    for i in range(N):
        if i == 0:
            Apow = A.copy()
        else:
            Apow = Apow @ A
        A_aug[i * nx:(i + 1) * nx, :] = Apow

        # B_aug の行 i
        A_pow = np.eye(nx, dtype=np.float32)  # A^0
        for j in range(i, -1, -1):
            B_aug[i * nx:(i + 1) * nx, j * nu:(j + 1) * nu] = A_pow @ B
            A_pow = A_pow @ A

    return A_aug, B_aug


def build_block_weights(Q, R, Qf):
    """Q_blk, R_blk を C++と同じロジックで作成"""
    nx, nu = Q.shape[0], R.shape[0]
    nxN = nx * N
    nuN = nu * N

    Q_blk = np.zeros((nxN, nxN), dtype=np.float32)
    R_blk = np.zeros((nuN, nuN), dtype=np.float32)

    # まず Q で埋める→後で終端重みだけ Qf に差し替えでもいいが
    # C++ の最終版と同様に直接 i==N-1 に Qf を入れる
    for i in range(N):
        Qi = Qf if i == N - 1 else Q
        Q_blk[i * nx:(i + 1) * nx, i * nx:(i + 1) * nx] = Qi
        R_blk[i * nu:(i + 1) * nu, i * nu:(i + 1) * nu] = R

    return Q_blk, R_blk


def build_cost_matrices(A, B, Q, R, Qf):
    """H, F_mpc, BtQ をまとめて構築"""
    A_aug, B_aug = build_prediction_matrices(A, B)
    Q_blk, R_blk = build_block_weights(Q, R, Qf)

    H = B_aug.T @ Q_blk @ B_aug + R_blk
    Fmpc = B_aug.T @ Q_blk @ A_aug
    BtQ = B_aug.T @ Q_blk

    return H.astype(np.float32), Fmpc.astype(np.float32), BtQ.astype(np.float32), A_aug, B_aug, Q_blk, R_blk


def build_constraints(A_aug, B_aug):
    """Gx, Gu, h_vec を C++と同じロジックで構築"""
    nx = NX
    nu = NU
    nxN = nx * N
    nuN = nu * N

    # 1ステップ分の状態・入力制約
    x_min = np.array([-0.5, -0.5, -5.0, -5.0, -3.0], dtype=np.float32)
    x_max = np.array([ 0.5,  0.5,  5.0,  5.0,  3.0], dtype=np.float32)

    # 現状コード（u は 0〜0.7）
    u_min = np.array([0.0, 0.0, 0.0], dtype=np.float32)
    u_max = np.array([0.7, 0.7, 0.7], dtype=np.float32)

    # スタック
    x_min_stack = np.tile(x_min, N)
    x_max_stack = np.tile(x_max, N)
    u_min_stack = np.tile(u_min, N)
    u_max_stack = np.tile(u_max, N)

    m_state = 2 * nxN
    m_input = 2 * nuN
    m = m_state + m_input

    Gx = np.zeros((m, nx), dtype=np.float32)
    Gu = np.zeros((m, nuN), dtype=np.float32)
    h_vec = np.zeros((m,), dtype=np.float32)

    # 状態上限: A_aug x0 + B_aug U <= x_max_stack
    Gx[0:nxN, :] = A_aug
    Gu[0:nxN, :] = B_aug
    h_vec[0:nxN] = x_max_stack

    # 状態下限: -A_aug x0 - B_aug U <= -x_min_stack
    Gx[nxN:2 * nxN, :] = -A_aug
    Gu[nxN:2 * nxN, :] = -B_aug
    h_vec[nxN:2 * nxN] = -x_min_stack

    offset = 2 * nxN

    # 入力上限: U <= u_max_stack
    Gu[offset:offset + nuN, 0:nuN] = np.eye(nuN, dtype=np.float32)
    h_vec[offset:offset + nuN] = u_max_stack
    offset += nuN

    # 入力下限: -U <= -u_min_stack
    Gu[offset:offset + nuN, 0:nuN] = -np.eye(nuN, dtype=np.float32)
    h_vec[offset:offset + nuN] = -u_min_stack

    return Gx, Gu, h_vec


# ========= ADMM 用行列 P, L =========

def build_admm_matrices(H, Gu):
    """P_admm とその Cholesky 因子 L を構築"""

    P = H + RHO_ADMM * (Gu.T @ Gu)
    # SPD のはずなので Cholesky
    L = np.linalg.cholesky(P).astype(np.float32)

    return P.astype(np.float32), L


# ========= C 配列出力 =========

def to_c_array(name, arr, cols_per_line=8):
    """
    Eigen と同じ column-major で flatten して
    const float name[...] = {...}; を返す
    """
    # 列優先（Fortran order）で flatten
    flat = np.array(arr, dtype=np.float32, order='F').ravel(order='F')
    lines = []
    lines.append(f"const float {name}[{flat.size}] = {{")

    for i, v in enumerate(flat):
        end = "," if i < flat.size - 1 else ""
        lines.append(f"  {v:.9e}f{end}")
        if (i + 1) % cols_per_line == 0 and i < flat.size - 1:
            lines[-1] += ""  # 改行だけ入れたいならここで制御

    lines.append("};")
    return "\n".join(lines)


def generate_header(filename="mpc_matrices.h"):
    A, B, Q, R, Qf = build_system_matrices()
    H, Fmpc, BtQ, A_aug, B_aug, Q_blk, R_blk = build_cost_matrices(A, B, Q, R, Qf)
    Gx, Gu, h_vec = build_constraints(A_aug, B_aug)
    P, L = build_admm_matrices(H, Gu)

    nxN = NX * N
    nuN = NU * N
    m_state = 2 * nxN
    m_input = 2 * nuN
    M = m_state + m_input

    with open(filename, "w", encoding="utf-8") as f:
        f.write("#pragma once\n\n")
        f.write("// Auto-generated by generate_mpc_matrices.py\n")
        f.write("// NX, NU, N は C++ 側と必ず一致させること\n\n")
        f.write(f"#define NX  {NX}\n")
        f.write(f"#define NU  {NU}\n")
        f.write(f"#define N   {N}\n")
        f.write(f"#define NXN (NX * N)\n")
        f.write(f"#define NUN (NU * N)\n")
        f.write(f"#define M   {M}\n")
        f.write(f"#define RHO_ADMM {RHO_ADMM}f\n\n")

        f.write("// Cost matrices\n")
        f.write(to_c_array("H_data",    H))
        f.write("\n\n")
        f.write(to_c_array("Fmpc_data", Fmpc))
        f.write("\n\n")
        f.write(to_c_array("BtQ_data",  BtQ))
        f.write("\n\n")

        f.write("// Constraint matrices\n")
        f.write(to_c_array("Gx_data",   Gx))
        f.write("\n\n")
        f.write(to_c_array("Gu_data",   Gu))
        f.write("\n\n")
        # h_vec はベクトルなので 1D で flatten すれば OK（row/col 意味なし）
        f.write(to_c_array("h_data",    h_vec.reshape(-1, 1)))
        f.write("\n\n")

        f.write("// ADMM P = H + rho * GuᵀGu の Cholesky factor L (P = L * Lᵀ)\n")
        f.write(to_c_array("L_data",    L))
        f.write("\n")

    print(f"Generated {filename}")


if __name__ == "__main__":
    generate_header()
