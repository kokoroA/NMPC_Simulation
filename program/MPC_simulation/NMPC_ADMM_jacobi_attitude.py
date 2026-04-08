import sympy as sp

# 1. 状態変数 X (5個: 姿勢制御用サブシステム)
#    u, v, w, Ze は姿勢・角速度の方程式に現れないため除外
p, q, r = sp.symbols('p q r', real=True)
phi, theta = sp.symbols('phi theta', real=True)

# 入力はモーター推力 T (4個)
T_fr, T_fl, T_rr, T_rl = sp.symbols('T_fr T_fl T_rr T_rl', real=True)

# 2. 定数パラメータ
Cp, Cq, Cr = sp.symbols('Cp Cq Cr', real=True)
Ix, Iy, Iz, l = sp.symbols('Ix Iy Iz l', real=True)
C_Q_T = sp.symbols('C_Q_T', real=True)  # 反トルクと推力の比 (CQ / CT)

# 3. 中間変数 (反トルク Q を推力 T に比例すると近似)
Q_fr = C_Q_T * T_fr
Q_fl = C_Q_T * T_fl
Q_rr = C_Q_T * T_rr
Q_rl = C_Q_T * T_rl

# 4. 運動方程式ベクトル f (5次元: 姿勢制御サブシステム)
#    ・角速度 (p, q, r) の方程式は u,v,w,Ze に無依存
#    ・オイラー角 (phi, theta) の方程式も u,v,w,Ze に無依存
f = sp.Matrix([
    # 角速度 p ドット
    (0.5*l*(T_fl+T_rl-T_fr-T_rr))/Ix - ((Iz-Iy)*q*r)/Ix - (Cp*p*sp.Abs(p))/Ix,
    # 角速度 q ドット
    (0.5*l*(T_fr+T_fl-T_rr-T_rl))/Iy - ((Ix-Iz)*r*p)/Iy - (Cq*q*sp.Abs(q))/Iy,
    # 角速度 r ドット
    (Q_fr+Q_rl-Q_fl-Q_rr)/Iz - ((Iy-Ix)*p*q)/Iz - (Cr*r*sp.Abs(r))/Iz,
    # phi ドット (オイラー角レート)
    p + q*sp.sin(phi)*sp.tan(theta) + r*sp.cos(phi)*sp.tan(theta),
    # theta ドット (オイラー角レート)
    q*sp.cos(phi) - r*sp.sin(phi),
])

# 状態ベクトル X (5次元) と 入力ベクトル U (4次元)
X = sp.Matrix([p, q, r, phi, theta])
U = sp.Matrix([T_fr, T_fl, T_rr, T_rl])

# 5. ヤコビ行列の計算 (解析的微分)
print("Calculating Jacobians (5x5 A, 5x4 B) ...")
A_sym = f.jacobian(X)  # 5x5 行列
B_sym = f.jacobian(U)  # 5x4 行列
print("Done.")

# 6. ヘッダファイルへ出力
NX = 5
NU = 4
output_path = "nmpc_jacobi_attitude.h"

# 引数リスト (状態 + 入力 + パラメータ)
state_args   = ', '.join(f'double {s}' for s in X)
input_args   = ', '.join(f'double {s}' for s in U)
param_args   = 'double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T'

lines = []
lines.append("#pragma once")
lines.append("")
lines.append("/* 姿勢制御用 NMPC ヤコビアン (5次元サブシステム)")
lines.append("   状態: [p, q, r, phi, theta]")
lines.append("   入力: [T_fr, T_fl, T_rr, T_rl] */")
lines.append("")
lines.append("#define NMPC_ATT_NX 5")
lines.append("#define NMPC_ATT_NU 4")
lines.append("")

# A行列 (5x5)
lines.append("/* --- 連続時間 状態ヤコビアン A (5x5) --- */")
lines.append(f"static inline void calc_jacobian_A_att(double *A, {state_args}, {input_args}, {param_args}) {{")
for i in range(NX):
    for j in range(NX):
        expr = A_sym[i, j]
        if expr != 0:
            lines.append(f"    A[{i} * {NX} + {j}] = {sp.ccode(expr)};")
lines.append("}")
lines.append("")

# B行列 (5x4)
lines.append("/* --- 連続時間 入力ヤコビアン B (5x4) --- */")
lines.append(f"static inline void calc_jacobian_B_att(double *B, {state_args}, {input_args}, {param_args}) {{")
for i in range(NX):
    for j in range(NU):
        expr = B_sym[i, j]
        if expr != 0:
            lines.append(f"    B[{i} * {NU} + {j}] = {sp.ccode(expr)};")
lines.append("}")
lines.append("")

with open(output_path, "w") as f_out:
    f_out.write("\n".join(lines))

print(f"Written to {output_path}")
