import casadi as ca
import numpy as np

# モデルパラメータ
Ix = 6.1e-3
Iy = 6.53e-3
Iz = 1.16e-2

# 状態・入力シンボル定義
x = ca.MX.sym('x', 6)  # φ, θ, ψ, p, q, r
u = ca.MX.sym('u', 3)  # δP, δQ, δR

# 状態展開
phi, theta, psi, p, q, r = x[0], x[1], x[2], x[3], x[4], x[5]
dP, dQ, dR = u[0], u[1], u[2]

# 状態方程式 f(x,u)
phi_dot = p + q*ca.sin(phi)*ca.tan(theta) + r*ca.cos(phi)*ca.tan(theta)
theta_dot = q*ca.cos(phi) - r*ca.sin(phi)
psi_dot = (q*ca.sin(phi) + r*ca.cos(phi)) / ca.cos(theta)

p_dot = (dP - (Iz - Iy)*q*r) / Ix
q_dot = (dQ - (Ix - Iz)*r*p) / Iy
r_dot = (dR - (Iy - Ix)*p*q) / Iz

f = ca.vertcat(phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot)

# ヤコビアンでA, Bを計算
A = ca.jacobian(f, x)
B = ca.jacobian(f, u)

# CasADi関数として定義
A_fun = ca.Function("A_fun", [x, u], [A])
B_fun = ca.Function("B_fun", [x, u], [B])

# ホバリング点で数値代入（x=0, u=0）
x0 = np.zeros(6)
u0 = np.zeros(3)
A_val = A_fun(x0, u0)
B_val = B_fun(x0, u0)

print("A =\n", np.array(A_val))
print("B =\n", np.array(B_val))
