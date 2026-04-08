# mpc_export.py
import numpy as np
from pathlib import Path
from textwrap import fill

def build_blocks(A, B, Q, R, N):
    nx, nu = A.shape[0], B.shape[1]
    nxN, nuN = nx*N, nu*N

    # --- Q_blk, R_blk（ブロック対角：必要なら出力、不要ならスキップ可能）
    Q_blk = np.kron(np.eye(N, dtype=np.float32), Q.astype(np.float32))
    R_blk = np.kron(np.eye(N, dtype=np.float32), R.astype(np.float32))

    # --- A_aug（先頭=A^1, …, A^N）
    A_aug = np.zeros((nxN, nx), dtype=np.float32)
    Apow = A.astype(np.float32).copy()
    for i in range(N):
        if i == 0:
            Apow[...] = A
        else:
            Apow = Apow @ A
        A_aug[i*nx:(i+1)*nx, :] = Apow

    # --- B_aug（(i,j)ブロック = A^{i-j} B, j<=i）
    B_aug = np.zeros((nxN, nuN), dtype=np.float32)
    for i in range(N):
        A_pow = np.eye(nx, dtype=np.float32)
        for j in range(i, -1, -1):
            B_aug[i*nx:(i+1)*nx, j*nu:(j+1)*nu] = A_pow @ B
            A_pow = A_pow @ A

    # --- H, F
    H = (B_aug.T @ Q_blk @ B_aug + R_blk).astype(np.float32)
    F = (B_aug.T @ Q_blk @ A_aug).astype(np.float32)

    # --- B^T（ランタイムで rhs を作る用。BtQ 常駐は不要）
    Bt = B_aug.T.astype(np.float32)

    return Q_blk, R_blk, A_aug, B_aug, H, F, Bt

def c_array(name, arr, per_line=8):
    flat = arr.flatten(order='C')  # Row-major で出力
    elems = ", ".join(f"{x:.8g}f" for x in flat)
    # 適当に折り返し
    lines = fill(elems, width=10000).split(", ")
    out = []
    line = []
    for i, tok in enumerate(lines, 1):
        line.append(tok)
        if i % per_line == 0:
            out.append(", ".join(line))
            line = []
    if line: out.append(", ".join(line))
    return f"alignas(16) const float {name}[] = {{\n  " + ",\n  ".join(out) + "\n};\n"

def write_headers(outdir, N, nx, nu, H, F, Bt, Q_blk=None, R_blk=None):
    outdir = Path(outdir)
    outdir.mkdir(parents=True, exist_ok=True)
    nuN, nxN = nu*N, nx*N

    # ヘッダ
    h = f"""#pragma once
#include <stdint.h>
#define MPC_N {N}
#define MPC_NX {nx}
#define MPC_NU {nu}
#define MPC_NX_N (MPC_NX*MPC_N)
#define MPC_NU_N (MPC_NU*MPC_N)

// 配列は Row-Major で格納されています。
// Eigen では Eigen::Map<const Matrix<float, Dynamic, Dynamic, RowMajor>> でマップしてください。

extern const float H_data[];   // size = MPC_NU_N * MPC_NU_N
extern const float F_data[];   // size = MPC_NU_N * MPC_NX
extern const float Bt_data[];  // size = MPC_NU_N * MPC_NX_N
"""
    if Q_blk is not None:
        h += "extern const float Qblk_data[]; // size = MPC_NX_N * MPC_NX_N\n"
    if R_blk is not None:
        h += "extern const float Rblk_data[]; // size = MPC_NU_N * MPC_NU_N\n"
    (outdir/"mpc_tables.h").write_text(h)

    # 実体（.cpp）
    cpp = '#include "mpc_tables.h"\n#include <cstddef>\n'
    cpp += c_array("H_data", H)
    cpp += c_array("F_data", F)
    cpp += c_array("Bt_data", Bt)
    if Q_blk is not None:
        cpp += c_array("Qblk_data", Q_blk)
    if R_blk is not None:
        cpp += c_array("Rblk_data", R_blk)
    (outdir/"mpc_tables.cpp").write_text(cpp)

def main():
    # === あなたの現在の設定 ===
    dt = 0.0025
    nx, nu = 5, 3
    # 先頭＝A（あなたのコードと一致）
    A = np.array([
        [1,0,dt,0,0],
        [0,1,0,dt,0],
        [0,0,1,0,0],
        [0,0,0,1,0],
        [0,0,0,0,1],
    ], dtype=np.float32)

    B = np.array([
        [0,0,0],
        [0,0,0],
        [276.9*dt, 0, 0],
        [0, 276.9*dt, 0],
        [0, 0, 304.6*dt],
    ], dtype=np.float32)

    Q = np.eye(nx, dtype=np.float32)
    Q[0,0]=30.0; Q[1,1]=30.0; Q[2,2]=0.1; Q[3,3]=0.1; Q[4,4]=1.0
    q_c = 0.1
    Q[0,2]=Q[2,0]=q_c
    Q[1,3]=Q[3,1]=q_c

    R = np.eye(nu, dtype=np.float32)  # 1,1,1

    # === 任意の N で出力 ===
    N = 30  # 必要に応じて変更
    Q_blk, R_blk, A_aug, B_aug, H, F, Bt = build_blocks(A,B,Q,R,N)

    # ざっくり容量を表示
    def kib(x): return x.nbytes/1024
    print(f"N={N}  nxN={nx*N}  nuN={nu*N}")
    print(f"H:   {H.shape}  ~{kib(H):.1f} KiB")
    print(f"F:   {F.shape}  ~{kib(F):.1f} KiB")
    print(f"Bt:  {Bt.shape} ~{kib(Bt):.1f} KiB")
    print(f"Qbk: {Q_blk.shape} ~{kib(Q_blk):.1f} KiB  (出力しない選択も可)")
    print(f"Rbk: {R_blk.shape} ~{kib(R_blk):.1f} KiB  (出力しない選択も可)")

    # === 出力（Q_blk/R_blkは None にすると出力しない）===
    write_headers(outdir="out_mpc", N=N, nx=nx, nu=nu, H=H, F=F, Bt=Bt,
                  Q_blk=None, R_blk=None)

if __name__ == "__main__":
    main()
