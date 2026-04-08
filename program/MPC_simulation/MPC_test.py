import casadi as ca
import numpy as np
from discritization_matrix_exp import discretize_matrix_exponential
from discritization_euler import discretize_euler
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

# === 状態方程式のA行列とB行列 ===
A = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
])

B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [1/Ix, 0, 0],
    [0, 1/Iy, 0],
    [0, 0, 1/Iz]
])

#euler法での離散化
# A_d = np.eye(nx) + dt * A
# B_d = dt * B

# === 指数関数法での離散化 ===
# A_d, B_d = discretize_matrix_exponential(A, B, dt)
A_d, B_d = discretize_euler(A, B, dt)  # euler法での離散化も可能
print("A_d:", A_d)
print("B_d:", B_d)
# === CasADi関数定義 ===
x = ca.MX.sym('x', nx) #状態ベクトル（シンボル）定義
u = ca.MX.sym('u', nu) #入力ベクトル（シンボル）定義
x_next = ca.mtimes(A_d, x) + ca.mtimes(B_d, u) #離散状態遷移式
f = ca.Function('f', [x, u], [x_next]) #状態遷移式f(x, u) をCasADi関数として定義

# === 最適化変数 ===
#状態変数を表すシンボリック行列.Nステップ先までの状態遷移が格納される
X = ca.MX.sym('X', nx, N+1)

#制御入力変数を表すシンボリック行列.Nステップ先までの制御入力が格納される
#制御入力はNステップ分あるので、Uの列数はN
U = ca.MX.sym('U', nu, N)

#パラメータ（動的に変わるが最適化対象ではない)は P にまとめて渡す必要がある
P = ca.MX.sym('P', 2*nx)  # [x0, x_ref]
Q = np.diag([100, 100, 10, 0.1, 0.1, 0.1]) #状態の偏差にかける重み行列
R = np.diag([1, 1, 1]) #入力の大きさにかけるペナルティ行列**（3×3）

# === コストと制約構築 ===
cost = 0
g = []

#予測ホライゾンN回ループ
#状態遷移とコスト関数の定義を行う
for k in range(N):
    xk = X[:,k] #ステップ k における状態（6次元）
    uk = U[:,k] #ステップ k における入力（3次元）
    x_ref = P[nx:]
    #目的関数（コスト関数）
    cost += ca.mtimes([(xk - x_ref).T, Q, (xk - x_ref)]) + ca.mtimes([uk.T, R, uk])
    #離散時間の状態遷移関数を使って次状態を予測
    x_next = f(xk, uk)
    #次のステップの状態とモデルから予測された次状態の差を0にするように等式制約を追加
    g.append(X[:,k+1] - x_next)#CasADiにとって「等式制約 = 0 の形」が必要

# 初期状態制約
#MPCが未来の状態と入力を最適化するとき、その出発点となる現在の状態x0をMPCが予測する最初の状態X[:,0]に強制的に一致させる制約です。
#MPCは「xの系列（X[:,0]～X[:,N]）」を最適化しますが、出発点（X[:,0]）がズレていたら、全体の予測が崩壊します。
#X[:, 0] : MPCが最適化しようとしている最初の状態（ステップ0）
#P[:nx] : 実際に観測された現在の状態（6次元）
#式にすると、X[:, 0]−P[:nx]=0（等式制約）
# g は状態遷移制約のリスト（X[k+1] = f(X[k], U[k])）
# 通常、g = [x1 - f(x0, u0), x2 - f(x1, u1), ...] のように並ぶ
# そこに x0 - x_real を「一番最初の制約」として明示的に追加する
g.insert(0, X[:,0] - P[:nx]) 

# === 最適化問題定義 ===
#最適化変数 : CasADiの最適化ソルバに渡すために、行列Xと行列Uをすべて1次元のベクトルに変形し、縦に連結（vertcat)
OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
#今回の最適化問題
#f:目的関数、x:最適化変数、g:制約式(等式制約：状態遷移や初期状態など)、p:パラメータ(実行時に外部から与える)
nlp_prob = {'f': cost, 'x': OPT_variables, 'g': ca.vertcat(*g), 'p': P}

# === ソルバ設定 ===
#ログ、実行時間出力設定
opts = {'ipopt.print_level': 0, 'print_time': 0}
#CasADiの非線形最適化ソルバ（nlpsol）を定義
#名前：'solver'
#使用アルゴリズム：'ipopt'（内点法）
#最適化問題 : nlp_prob
#出力設定 : opts
solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)

# === MPC制御関数 === 
#現在の状態x0と目標状態x_refを入力
#最適な最初のトルク指令 `u0` を出力
def mpc_control(x0, x_ref):
    args = {
        'p': np.concatenate((x0, x_ref)), #定数パラメータ。x0（現在の状態),x_ref（目標状態）
        'x0': np.zeros((nx*(N+1) + nu*N, 1)), #初期推定解 `x0` を0で初期化（状態系列 + 入力系列）
        'lbg': np.zeros(nx*(N+1)), #状態遷移や初期状態の等式制約の上下限を0にする
        'ubg': np.zeros(nx*(N+1)) #状態遷移や初期状態の等式制約の上下限を0にする
    }
    #最適化問題を解いて `sol` に結果を格納
    sol = solver(**args)
    U_opt = sol['x'][-nu*N:]  # 最後のU
    u0 = np.array(U_opt[:nu]).flatten()
    return u0 #[deltaP,deltaQ,deltaR]

# === テスト ===
x = np.array([0.5, 0, 0, 0, 0, 0])  # 初期状態（φ, θ, ψ, p, q, r） 
x_ref = np.array([0, 0, 0, 0, 0, 0])  # 目標状態
runge_x_plus.phi = 0.5
u0 = mpc_control(x, x_ref)
print("最適化された初期入力:", u0)

# ログ用
X_log = []
U_log = []
for t in range(steps):

    runge_x_plus.X.append(runge_x_plus.Xe)
    runge_x_plus.Y.append(runge_x_plus.Ye)
    runge_x_plus.Z.append(-1*runge_x_plus.Ze)
    runge_x_plus.P.append(runge_x_plus.p)
    runge_x_plus.Q.append(runge_x_plus.q)
    runge_x_plus.R.append(-1*runge_x_plus.r)

    # === MPCで制御入力を計算 ===
    u = mpc_control(x, x_ref)  # ← ここで最適化

    # === モデルで次の状態を計算 ===
    #このxは4次のノルンゲクッタで更新した非線形の運動方程式の結果を格納
    #センサノイズを入れる必要あり
    # シミュレーション用の x_{k+1} = A_d x_k + B_d u_k
    # x = A_d @ x + B_d @ u  # ← ここは「真の」状態遷移
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
    # === ログ保存 ===
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