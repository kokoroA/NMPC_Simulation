import sympy as sp

# 1. 状態変数 X (15個) と 入力変数 U (4個) の定義
u, v, w = sp.symbols('u v w', real=True)
p, q, r = sp.symbols('p q r', real=True)
phi, theta, psi = sp.symbols('phi theta psi', real=True)
w_fr, w_fl, w_rr, w_rl = sp.symbols('w_fr w_fl w_rr w_rl', real=True)
Xe, Ye, Ze = sp.symbols('Xe Ye Ze', real=True)

e1, e2, e3, e4 = sp.symbols('e1 e2 e3 e4', real=True)

# 2. 定数パラメータの定義
g, m, Cu, Cv, Cw = sp.symbols('g m Cu Cv Cw', real=True)
Cp, Cq, Cr = sp.symbols('Cp Cq Cr', real=True)
Ix, Iy, Iz, l = sp.symbols('Ix Iy Iz l', real=True)
K, Reg, Iz_m, Qf, CQ, D = sp.symbols('K Reg Iz_m Qf CQ D', real=True)
CT, CQ_aero = sp.symbols('CT CQ_aero', real=True) # 推力係数とトルク係数

# 3. 中間変数の展開 (推力と反トルクを角速度 w_** で表現)
Tfr = CT * w_fr**2
Tfl = CT * w_fl**2
Trr = CT * w_rr**2
Trl = CT * w_rl**2

Qfr = CQ_aero * w_fr**2
Qfl = CQ_aero * w_fl**2
Qrr = CQ_aero * w_rr**2
Qrl = CQ_aero * w_rl**2

# 4. 運動方程式ベクトルの定義
f = sp.Matrix([
    # 並進速度 (u, v, w)
    -g*sp.sin(theta) - (Cu*u*sp.Abs(u))/m - q*w + r*v,
    g*sp.cos(theta)*sp.sin(phi) - (Cv*v*sp.Abs(v))/m - r*u + q*w,
    g*sp.cos(theta)*sp.cos(phi) - (Cw*w*sp.Abs(w))/m - (Tfr+Tfl+Trr+Trl)/m - q*v + q*u,
    
    # モーター角速度 (w_fr, w_fl, w_rr, w_rl)
    (K*e1)/(Reg*Iz_m) - Qf/Iz_m - (CQ*w_fr**2)/Iz_m - ((D+K**2)/Reg)*w_fr/Iz_m,
    (K*e4)/(Reg*Iz_m) - Qf/Iz_m - (CQ*w_fl**2)/Iz_m - ((D+K**2)/Reg)*w_fl/Iz_m,
    (K*e2)/(Reg*Iz_m) - Qf/Iz_m - (CQ*w_rr**2)/Iz_m - ((D+K**2)/Reg)*w_rr/Iz_m,
    (K*e3)/(Reg*Iz_m) - Qf/Iz_m - (CQ*w_rl**2)/Iz_m - ((D+K**2)/Reg)*w_rl/Iz_m,

    # 角速度 (p, q, r)
    (0.5*l*(Tfl+Trl-Tfr-Trr))/Ix - ((Iz-Iy)*q*r)/Ix - (Cp*p*sp.Abs(p))/Ix,
    (0.5*l*(Tfr+Tfl-Trr-Trl))/Iy - ((Ix-Iz)*r*p)/Iy - (Cq*q*sp.Abs(q))/Iy,
    (Qfr+Qrl-Qfl-Qrr)/Iz - ((Iy-Ix)*p*q)/Iz - (Cr*r*sp.Abs(r))/Iz,

    # オイラー角 (phi, theta, psi)
    p + q*sp.sin(phi)*sp.tan(theta) + r*sp.cos(phi)*sp.tan(theta),
    q*sp.cos(phi) - r*sp.sin(phi),
    (q*sp.sin(phi) + r*sp.cos(phi))/sp.cos(theta),

    # 位置 (Xe, Ye, Ze)
    u*sp.cos(theta)*sp.cos(psi) + v*(sp.sin(phi)*sp.sin(theta)*sp.cos(psi) - sp.cos(phi)*sp.sin(psi)) + w*(sp.cos(phi)*sp.sin(theta)*(sp.cos(psi)+sp.sin(phi)*sp.sin(psi))),
    u*sp.cos(theta)*sp.sin(psi) + v*(sp.sin(phi)*sp.sin(theta)*sp.sin(psi) + sp.cos(phi)*sp.cos(psi)) + w*(sp.cos(phi)*sp.sin(theta)*sp.sin(psi) - sp.sin(phi)*sp.cos(psi)),
    -u*sp.sin(theta) + v*sp.sin(phi)*sp.cos(theta) + w*sp.cos(phi)*sp.cos(theta)
])

# 状態ベクトルと入力ベクトル
X = sp.Matrix([u, v, w, w_fr, w_fl, w_rr, w_rl, p, q, r, phi, theta, psi, Xe, Ye, Ze])
U = sp.Matrix([e1, e2, e3, e4])

# 5. ヤコビ行列の計算 (解析的微分)
print("Calculating Jacobians...")
A_sym = f.jacobian(X)  # 15 x 15 行列
B_sym = f.jacobian(U)  # 15 x 4  行列

# 6. Cヘッダーファイルとして出力
HEADER_NAME = "nmpc_jacobi.h"
NX = A_sym.rows
NU = U.rows

lines = []
lines.append("#ifndef NMPC_JACOBI_H")
lines.append("#define NMPC_JACOBI_H")
lines.append("")
lines.append("#include <math.h>")
lines.append("#include <string.h>")
lines.append("")
lines.append("// SymPyが出力する sign() 関数をC++で処理するための実装")
lines.append("inline double sign(double x) {")
lines.append("    if (x > 0.0) return 1.0;")
lines.append("    if (x < 0.0) return -1.0;")
lines.append("    return 0.0;")
lines.append("}")
lines.append("")
lines.append("#define NMPC_NX 15")
lines.append("#define NMPC_NU 4")
lines.append("")
lines.append("/* 連続時間ヤコビアン A(NX×NX), B(NX×NU) を埋める。A, B は呼び出し側で確保すること。 */")
lines.append("static inline void nmpc_jacobi_AB(")
lines.append("    double *A, double *B,")
lines.append("    double u, double v, double w, double w_fr, double w_fl, double w_rr, double w_rl,")
lines.append("    double p, double q, double r, double phi, double theta, double psi,")
lines.append("    double Xe, double Ye, double Ze,")
lines.append("    double g, double m, double Cu, double Cv, double Cw, double Cp, double Cq, double Cr,")
lines.append("    double Ix, double Iy, double Iz, double l, double K, double Reg, double Iz_m,")
lines.append("    double Qf, double CQ, double D, double CT, double CQ_aero)")
lines.append("{")
lines.append("    memset(A, 0, NMPC_NX * NMPC_NX * sizeof(double));")
lines.append("    memset(B, 0, NMPC_NX * NMPC_NU * sizeof(double));")
lines.append("")

for i in range(A_sym.rows):
    for j in range(A_sym.cols):
        if A_sym[i, j] != 0:
            c = sp.ccode(A_sym[i, j], strict=False)
            lines.append(f"    A[{i} * NMPC_NX + {j}] = {c};")

lines.append("")

for i in range(B_sym.rows):
    for j in range(B_sym.cols):
        if B_sym[i, j] != 0:
            c = sp.ccode(B_sym[i, j], strict=False)
            lines.append(f"    B[{i} * NMPC_NU + {j}] = {c};")

lines.append("}")
lines.append("")
lines.append("#endif /* NMPC_JACOBI_H */")
lines.append("")

with open(HEADER_NAME, "w", encoding="utf-8") as fout:
    fout.write("\n".join(lines))

print(f"Written: {HEADER_NAME}")