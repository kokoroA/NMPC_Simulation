import sympy as sp

# 1. 状態変数 X (5個: 姿勢制御用サブシステム)
#    u, v, w, Ze は姿勢・角速度の方程式に現れないため除外
p, q, r = sp.symbols('p q r', real=True)
phi, theta = sp.symbols('phi theta', real=True)

# 入力は仮想トルク (3個): roll/pitch/yaw 軸それぞれの合成トルク
# tau_phi   = 0.5*l*(T_fl + T_rl - T_fr - T_rr)
# tau_theta = 0.5*l*(T_fr + T_fl - T_rr - T_rl)
# tau_r     = C_Q_T*(T_fr + T_rl - T_fl - T_rr)
tau_phi, tau_theta, tau_r = sp.symbols('tau_phi tau_theta tau_r', real=True)

# 2. 定数パラメータ (アーム長 l, 反トルク比 C_Q_T は仮想トルクに吸収済みで不要)
Cp, Cq, Cr = sp.symbols('Cp Cq Cr', real=True)
Ix, Iy, Iz = sp.symbols('Ix Iy Iz', real=True)

# 3. 運動方程式ベクトル f (5次元: 姿勢制御サブシステム)
#    状態ベクトルの並びを X = [phi, theta, p, q, r]^T とする
#    非線形クロス項は 4入力版と同一、トルク入力のみ置換
f = sp.Matrix([
    # phi ドット (オイラー角レート)
    p + q*sp.sin(phi)*sp.tan(theta) + r*sp.cos(phi)*sp.tan(theta),
    # theta ドット (オイラー角レート)
    q*sp.cos(phi) - r*sp.sin(phi),
    # 角速度 p ドット
    tau_phi/Ix - ((Iz-Iy)*q*r)/Ix - (Cp*p*sp.Abs(p))/Ix,
    # 角速度 q ドット
    tau_theta/Iy - ((Ix-Iz)*r*p)/Iy - (Cq*q*sp.Abs(q))/Iy,
    # 角速度 r ドット
    tau_r/Iz - ((Iy-Ix)*p*q)/Iz - (Cr*r*sp.Abs(r))/Iz,
])

# 状態ベクトル X (5次元) と 入力ベクトル U (3次元)
X = sp.Matrix([phi, theta, p, q, r])
U = sp.Matrix([tau_phi, tau_theta, tau_r])

# 4. ヤコビ行列の計算 (解析的微分)
print("Calculating Jacobians (5x5 A, 5x3 B) ...")
A_sym = f.jacobian(X)  # 5x5 行列
B_sym = f.jacobian(U)  # 5x3 行列
print("Done.")

# 5. ヘッダファイルへ出力
NX = 5
NU = 3
output_path = "nmpc_jacobi_attitude_3input.h"

# 引数リスト (状態 + 入力 + パラメータ)
state_args = ', '.join(f'double {s}' for s in X)
input_args = ', '.join(f'double {s}' for s in U)
param_args = 'double Cp, double Cq, double Cr, double Ix, double Iy, double Iz'

lines = []
lines.append("#pragma once")
lines.append("")
lines.append("/* 姿勢制御用 NMPC ヤコビアン (5次元サブシステム, 3入力版)")
lines.append("   状態: [phi, theta, p, q, r]")
lines.append("   入力: [tau_phi, tau_theta, tau_r] (roll/pitch/yaw 仮想トルク)")
lines.append("   4入力版との違い: モーター推力 T_fr/fl/rr/rl を仮想トルクに置換済み")
lines.append("     tau_phi   = 0.5*l*(T_fl+T_rl-T_fr-T_rr)")
lines.append("     tau_theta = 0.5*l*(T_fr+T_fl-T_rr-T_rl)")
lines.append("     tau_r     = C_Q_T*(T_fr+T_rl-T_fl-T_rr) */")
lines.append("")
lines.append("#define NMPC_ATT_NX 5")
lines.append("#define NMPC_ATT_NU 3")
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

# B行列 (5x3)
lines.append("/* --- 連続時間 入力ヤコビアン B (5x3) --- */")
lines.append("/* 注: B は状態・入力に依存しない定数行列になります")
lines.append("       B = diag(1/Ix, 1/Iy, 1/Iz) (上 3x3) + 零行列 (下 2x3) */")
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
