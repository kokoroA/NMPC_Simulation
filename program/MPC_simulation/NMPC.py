import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import runge_x_plus

# === モデルパラメータ ===
nx = 6  # 状態次元（φ, θ, ψ, p, q, r）
nu = 3  # 入力次元（δP, δQ, δR）
N = 20  # 予測ホライゾン
dt = 0.01  # サンプリング周期（秒）
steps = 100  # シミュレーション時間ステップ数
Ix, Iy, Iz = 6.1e-3, 6.53e-3, 1.16e-2

# === NMPCの非線形状態遷移モデル ===
def drone_nonlinear_dynamics_casadi():
    x = ca.MX.sym('x', nx)
    u = ca.MX.sym('u', nu)

    phi, theta, psi, p, q, r = x[0], x[1], x[2], x[3], x[4], x[5]
    dP, dQ, dR = u[0], u[1], u[2]

    Cp, Cq, Cr = runge_x_plus.Cp, runge_x_plus.Cq, runge_x_plus.Cr

    phi_dot = p + q * ca.sin(phi) * ca.tan(theta) + r * ca.cos(phi) * ca.tan(theta)
    theta_dot = q * ca.cos(phi) - r * ca.sin(phi)
    psi_dot = (q * ca.sin(phi) + r * ca.cos(phi)) / ca.cos(theta)

    p_dot = (dP - (Iz - Iy) * q * r - ca.sign(p) * Cp * p**2) / Ix
    q_dot = (dQ - (Ix - Iz) * r * p - ca.sign(q) * Cq * q**2) / Iy
    r_dot = (dR - (Iy - Ix) * p * q - ca.sign(r) * Cr * r**2) / Iz

    x_dot = ca.vertcat(phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot)
    x_next = x + dt * x_dot
    return ca.Function('f', [x, u], [x_next])

# === MPC制御関数 ===
def mpc_control(x0, x_ref):
    f = drone_nonlinear_dynamics_casadi()

    X = ca.MX.sym('X', nx, N+1)
    U = ca.MX.sym('U', nu, N)
    P = ca.MX.sym('P', 2*nx)

    Q = np.diag([100, 100, 10, 0.1, 0.1, 0.1])
    R = np.diag([0.1, 0.1, 0.1])

    cost = 0
    g = []

    for k in range(N):
        xk = X[:, k]
        uk = U[:, k]
        x_ref_k = P[nx:]
        cost += ca.mtimes([(xk - x_ref_k).T, Q, (xk - x_ref_k)]) + ca.mtimes([uk.T, R, uk])
        x_next = f(xk, uk)
        g.append(X[:, k+1] - x_next)

    g.insert(0, X[:, 0] - P[:nx])
    OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
    nlp_prob = {'f': cost, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}

    opts = {'ipopt.print_level': 0, 'print_time': 0}
    solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

    args = {
        'p': np.concatenate((x0, x_ref)),
        'x0': np.zeros((nx*(N+1) + nu*N, 1)),
        'lbg': np.zeros(nx*(N+1)),
        'ubg': np.zeros(nx*(N+1))
    }

    sol = solver(**args)
    U_opt = sol['x'][-nu*N:]
    u0 = np.array(U_opt[:nu]).flatten()
    return u0

# === シミュレーション ===
x = np.array([0, 0, 0, 0, 0, 0])  # 初期状態（φ, θ, ψ, p, q, r） 
x_ref = np.array([0, 0, 0, 0, 0, 0])  # 目標状態
runge_x_plus.phi = 0
u0 = mpc_control(x, x_ref)
print("最適化された初期入力:", u0)

X_log, U_log = [], []
for t in range(steps):

    runge_x_plus.X.append(runge_x_plus.Xe)
    runge_x_plus.Y.append(runge_x_plus.Ye)
    runge_x_plus.Z.append(-1*runge_x_plus.Ze)
    runge_x_plus.P.append(runge_x_plus.p)
    runge_x_plus.Q.append(runge_x_plus.q)
    runge_x_plus.R.append(-1*runge_x_plus.r)

    x_ref = np.array([0.5, 0, 0, 0, 0, 0])  # 目標状態

    u = mpc_control(x, x_ref)
    
    # === モデルで次の状態を計算 ===
    #このxは4次のノルンゲクッタで更新した非線形の運動方程式の結果を格納
    #センサノイズを入れる必要あり
    runge_x_plus.D_list = np.array([[0],[u[0]],[u[1]],[u[2]]])
    runge_x_plus.e_list = runge_x_plus.A_val * runge_x_plus.B_inv @ runge_x_plus.C_inv @ runge_x_plus.D_list
    runge_x_plus.e1 = runge_x_plus.e0 + (runge_x_plus.e_list[0][0])
    runge_x_plus.e2 = runge_x_plus.e0 + (runge_x_plus.e_list[1][0])
    runge_x_plus.e3 = runge_x_plus.e0 + (runge_x_plus.e_list[2][0])
    runge_x_plus.e4 = runge_x_plus.e0 + (runge_x_plus.e_list[3][0])
    
    #---------モーターの角速度---------
    runge_x_plus.omega_fr = runge_x_plus.rk4(runge_x_plus.omega_dot_fr, runge_x_plus.omega_fr, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.omega_fl = runge_x_plus.rk4(runge_x_plus.omega_dot_fl, runge_x_plus.omega_fl, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.omega_rr = runge_x_plus.rk4(runge_x_plus.omega_dot_rr, runge_x_plus.omega_rr, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.omega_rl = runge_x_plus.rk4(runge_x_plus.omega_dot_rl, runge_x_plus.omega_rl, runge_x_plus.t, runge_x_plus.h)

    if runge_x_plus.omega_fr < 0: runge_x_plus.omega_fr = 0
    if runge_x_plus.omega_fl < 0: runge_x_plus.omega_fl = 0
    if runge_x_plus.omega_rr < 0: runge_x_plus.omega_rr = 0
    if runge_x_plus.omega_rl < 0: runge_x_plus.omega_rl = 0

    #---------トルク計算---------
    runge_x_plus.Qfr = runge_x_plus.CQ * runge_x_plus.omega_fr**2
    runge_x_plus.Qfl = runge_x_plus.CQ * runge_x_plus.omega_fl**2
    runge_x_plus.Qrr = runge_x_plus.CQ * runge_x_plus.omega_rr**2
    runge_x_plus.Qrl = runge_x_plus.CQ * runge_x_plus.omega_rl**2

    #---------推力計算---------
    runge_x_plus.Tfr = runge_x_plus.Ct * runge_x_plus.omega_fr**2
    runge_x_plus.Tfl = runge_x_plus.Ct * runge_x_plus.omega_fl**2
    runge_x_plus.Trr = runge_x_plus.Ct * runge_x_plus.omega_rr**2
    runge_x_plus.Trl = runge_x_plus.Ct * runge_x_plus.omega_rl**2

    #---------速度の計算---------
    runge_x_plus.u = runge_x_plus.rk4(runge_x_plus.u_dot, runge_x_plus.u, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.v = runge_x_plus.rk4(runge_x_plus.v_dot, runge_x_plus.v, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.w = runge_x_plus.rk4(runge_x_plus.w_dot, runge_x_plus.w, runge_x_plus.t, runge_x_plus.h)

    #---------角速度の計算---------
    runge_x_plus.p = runge_x_plus.rk4(runge_x_plus.p_dot, runge_x_plus.p, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.q = runge_x_plus.rk4(runge_x_plus.q_dot, runge_x_plus.q, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.r = runge_x_plus.rk4(runge_x_plus.r_dot, runge_x_plus.r, runge_x_plus.t, runge_x_plus.h)

    #---------角度の計算---------
    runge_x_plus.phi =   runge_x_plus.rk4(runge_x_plus.phi_dot, runge_x_plus.phi, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.theta = runge_x_plus.rk4(runge_x_plus.theta_dot, runge_x_plus.theta, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.psi =   runge_x_plus.rk4(runge_x_plus.psi_dot, runge_x_plus.psi, runge_x_plus.t, runge_x_plus.h)

    #---------位置の計算---------
    runge_x_plus.Xe = runge_x_plus.rk4(runge_x_plus.Xe_dot, runge_x_plus.Xe, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.Ye = runge_x_plus.rk4(runge_x_plus.Ye_dot, runge_x_plus.Ye, runge_x_plus.t, runge_x_plus.h)
    runge_x_plus.Ze = runge_x_plus.rk4(runge_x_plus.Ze_dot, runge_x_plus.Ze, runge_x_plus.t, runge_x_plus.h)


    x = np.array([runge_x_plus.phi,runge_x_plus.theta,runge_x_plus.psi,runge_x_plus.p,runge_x_plus.q,runge_x_plus.r])

    X_log.append(x.copy())
    U_log.append(u.copy())

# === プロット ===
# numpy配列へ変換
X_log = np.array(X_log)
U_log = np.array(U_log)
X_pos = np.array(runge_x_plus.X)
Y_pos = np.array(runge_x_plus.Y)
Z_pos = np.array(runge_x_plus.Z)
P_log = np.array(runge_x_plus.P)
Q_log = np.array(runge_x_plus.Q)
R_log = np.array(runge_x_plus.R)

# === 1. 3次元軌道の可視化 ===
fig = plt.figure(figsize=(10, 6))
ax = fig.add_subplot(111, projection='3d')
ax.plot(X_pos, Y_pos, Z_pos, label="Drone Path")
ax.set_title("3D Drone Trajectory")
ax.set_xlabel("X [m]")
ax.set_ylabel("Y [m]")
ax.set_zlabel("Z [m]")
ax.legend()
ax.grid()
plt.tight_layout()
plt.show()

# === 2. 角速度の可視化（p, q, r） ===
plt.figure(figsize=(10, 4))
plt.plot(P_log, label='p [rad/s]')
plt.plot(Q_log, label='q [rad/s]')
plt.plot(R_log, label='r [rad/s]')
plt.title("Angular Velocities (p, q, r)")
plt.xlabel("Time Step")
plt.ylabel("Angular Velocity [rad/s]")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# === 3. 姿勢角の可視化（φ, θ, ψ） ===
plt.figure(figsize=(10, 4))
plt.plot(X_log[:, 0], label="phi [rad]")
plt.plot(X_log[:, 1], label="theta [rad]")
plt.plot(X_log[:, 2], label="psi [rad]")
plt.title("Angles (φ, θ, ψ)")
plt.xlabel("Time Step")
plt.ylabel("Angle [rad]")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()

# === 4. トルク入力の可視化（δP, δQ, δR） ===
plt.figure(figsize=(10, 4))
plt.plot(U_log[:, 0], label="δP")
plt.plot(U_log[:, 1], label="δQ")
plt.plot(U_log[:, 2], label="δR")
plt.title("Control Inputs (Torques)")
plt.xlabel("Time Step")
plt.ylabel("Torque Command")
plt.legend()
plt.grid()
plt.tight_layout()
plt.show()