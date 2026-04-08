import sympy as sp

# 1. 状態変数 X (15個) と 入力変数 U (4個) の定義
u, v, w = sp.symbols('u v w')
p, q, r = sp.symbols('p q r')
phi, theta, psi = sp.symbols('phi theta psi')
w_fr, w_fl, w_rr, w_rl = sp.symbols('w_fr w_fl w_rr w_rl')
Xe, Ye, Ze = sp.symbols('Xe Ye Ze')

e1, e2, e3, e4 = sp.symbols('e1 e2 e3 e4')

# 2. 定数パラメータの定義
g, m, Cu, Cv, Cw = sp.symbols('g m Cu Cv Cw')
Cp, Cq, Cr = sp.symbols('Cp Cq Cr')
Ix, Iy, Iz, l = sp.symbols('Ix Iy Iz l')
K, Reg, Iz_m, Qf, CQ, D = sp.symbols('K Reg Iz_m Qf CQ D')
CT, CQ_aero = sp.symbols('CT CQ_aero') # 推力係数とトルク係数

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
A_sym = f.jacobian(X) # 15 x 15 行列
B_sym = f.jacobian(U) # 15 x 4  行列

# 6. C言語コードとして出力
print("\n/* --- 連続時間 状態ヤコビアン A (15x15) --- */")
for i in range(A_sym.rows):
    for j in range(A_sym.cols):
        if A_sym[i,j] != 0:
            print(f"A[{i} * 15 + {j}] = {sp.ccode(A_sym[i,j])};")

print("\n/* --- 連続時間 入力ヤコビアン B (15x4) --- */")
for i in range(B_sym.rows):
    for j in range(B_sym.cols):
        if B_sym[i,j] != 0:
            print(f"B[{i} * 4 + {j}] = {sp.ccode(B_sym[i,j])};")