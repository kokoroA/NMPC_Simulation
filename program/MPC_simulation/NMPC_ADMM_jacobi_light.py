import sympy as sp
import sys

# 1. 状態変数 X (9個) と 入力変数 U (4個) をすべて実数として定義
u, v, w = sp.symbols('u v w', real=True)
p, q, r = sp.symbols('p q r', real=True)
phi, theta = sp.symbols('phi theta', real=True)
Ze = sp.symbols('Ze', real=True)

# 入力は直接モーター推力 T とする
T_fr, T_fl, T_rr, T_rl = sp.symbols('T_fr T_fl T_rr T_rl', real=True)

# 2. 定数パラメータ
g, m, Cu, Cv, Cw = sp.symbols('g m Cu Cv Cw', real=True)
Cp, Cq, Cr = sp.symbols('Cp Cq Cr', real=True)
Ix, Iy, Iz, l = sp.symbols('Ix Iy Iz l', real=True)
C_Q_T = sp.symbols('C_Q_T', real=True)  # 反トルクと推力の比 (CQ / CT)

# 3. 中間変数 (反トルク Q を推力 T に比例すると近似)
Q_fr = C_Q_T * T_fr
Q_fl = C_Q_T * T_fl
Q_rr = C_Q_T * T_rr
Q_rl = C_Q_T * T_rl

# 4. 運動方程式ベクトル f (9次元)
f = sp.Matrix([
    # 並進速度 (u, v, w)
    -g*sp.sin(theta) - (Cu*u*sp.Abs(u))/m - q*w + r*v,
    g*sp.cos(theta)*sp.sin(phi) - (Cv*v*sp.Abs(v))/m - r*u + q*w,
    g*sp.cos(theta)*sp.cos(phi) - (Cw*w*sp.Abs(w))/m - (T_fr+T_fl+T_rr+T_rl)/m - q*v + q*u,

    # 角速度 (p, q, r)
    (0.5*l*(T_fl+T_rl-T_fr-T_rr))/Ix - ((Iz-Iy)*q*r)/Ix - (Cp*p*sp.Abs(p))/Ix,
    (0.5*l*(T_fr+T_fl-T_rr-T_rl))/Iy - ((Ix-Iz)*r*p)/Iy - (Cq*q*sp.Abs(q))/Iy,
    (Q_fr+Q_rl-Q_fl-Q_rr)/Iz - ((Iy-Ix)*p*q)/Iz - (Cr*r*sp.Abs(r))/Iz,

    # オイラー角 (phi, theta)  ※psiは削除
    p + q*sp.sin(phi)*sp.tan(theta) + r*sp.cos(phi)*sp.tan(theta),
    q*sp.cos(phi) - r*sp.sin(phi),

    # 高度 Ze (地球座標系でのZ方向速度)
    -u*sp.sin(theta) + v*sp.sin(phi)*sp.cos(theta) + w*sp.cos(phi)*sp.cos(theta)
])

# 状態ベクトル X (9次元) と 入力ベクトル U (4次元)
X = sp.Matrix([u, v, w, p, q, r, phi, theta, Ze])
U = sp.Matrix([T_fr, T_fl, T_rr, T_rl])

# 5. ヤコビ行列の計算 (解析的微分)
print("Calculating Jacobians (9x9 A, 9x4 B) ...")
A_sym = f.jacobian(X)  # 9x9 行列
B_sym = f.jacobian(U)  # 9x4 行列
print("Done.")

# 6. ヘッダファイルへ出力
NX = 9
NU = 4
output_path = "nmpc_jacobi_light.h"

lines = []
lines.append("#pragma once")
lines.append("")
lines.append("#define NMPC_NX 9")
lines.append("#define NMPC_NU 4")
lines.append("")
lines.append("/* --- 連続時間 状態ヤコビアン A (9x9) --- */")
lines.append(f"static inline void calc_jacobian_A(double *A, double {', '.join(str(s) for s in X)}, double {', '.join(str(s) for s in U)}, double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {{")
for i in range(NX):
    for j in range(NX):
        expr = A_sym[i, j]
        if expr != 0:
            lines.append(f"    A[{i} * {NX} + {j}] = {sp.ccode(expr)};")
lines.append("}")
lines.append("")
lines.append("/* --- 連続時間 入力ヤコビアン B (9x4) --- */")
lines.append(f"static inline void calc_jacobian_B(double *B, double {', '.join(str(s) for s in X)}, double {', '.join(str(s) for s in U)}, double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr, double Ix, double Iy, double Iz, double l, double C_Q_T) {{")
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
