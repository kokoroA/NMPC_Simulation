import matplotlib
matplotlib.use('Qt5Agg')
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
import threading
import time
import math
from discritization_euler import discretize_euler
import casadi as ca

start_time = time.time()  # 開始時間

#---------パラメータ設定---------
g = 9.81                                    # 重力加速度(m/s)
m = 0.71                                    # 機体質量(kg)
l = 0.18                                  #モーター間の距離(m)
# l = 0.254558                                #モーター間の距離(m)
K = 3.28e-3 #3.28e-3                        #トルク定数
Km = 8.73e-8                                #トルク定数
Ct = 8.3e-7                                 #推力係数
CQ = 3.0e-8#4.53e-14#3.0e-8                 #トルク係数
D = 0#15.01e-6#0                            #動粘性抵抗係数
Qf = 0 #4e-6                                #摩擦トルク
Reg = 0.12                                  #巻線抵抗値
x1, x2, x3, x4 = 0.5*l, 0.5*l, 0.5*l, 0.5*l #重心からモーターの距離
y1, y2, y3, y4 = 0.5*l, 0.5*l, 0.5*l, 0.5*l #重心からモーターの距離
Ix, Iy, Iz = 6.1e-3, 6.53e-3, 1.16e-2       # 慣性モーメント
Ix_m, Iy_m, Iz_m = 5.02e-6, 5.02e-6, 8.12e-6# モーター＋プロペラの慣性モーメント
Cu,Cv,Cw = 0.2,0.2,0.9                     #空力係数 0.1,0.025,0.75    
Cp,Cq,Cr = 0.01,0.025,0.005                 #空力係数 0.01,0.025,0.75    
Tfr,Tfl,Trr,Trl = 0,0,0,0                   #推力(下向き正)
Qfr,Qfl,Qrr,Qrl = 0,0,0,0                   #モータートルク
u, v, w = 0.0, 0.0, 0.0                     #速度
omega_fr,omega_fl,omega_rr,omega_rl= 0,0,0,0#モーターの角速度
p, q, r = 0.0, 0.0, 0.0                     #角速度
phi, theta, psi = 0.0, 0.0, 0.0             # 初期オイラー角
Xe, Ye, Ze = 0.0, 0.0, 0.0                  # 位置
T_ref,deltaP,deltaQ,deltaR = 0,0,0,0        #スロットル、ロール、ピッチ、ヨー
Ref_p,Ref_q,Ref_r = 0,0,0                   #目標角速度
Ref_phi,Ref_theta,Ref_psi = 0,0,0           #目標角度
Ref_z,Ref_w,deltaw = 0,0,0                  #目標高さ、高さ方向の速度,高さ方向の速度目標値
q0,q1,q2,q3 = 1,0,0,0                       #クォータ二オン
norm_num = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
q0 = q0 / norm_num
q1 = q1 / norm_num
q2 = q2 / norm_num
q3 = q3 / norm_num
val = []                   
normalized_list = []
#---------釣り合いの推力---------                 
T_balance = m*g                             #釣り合いの推力 m*g                       
omega_zero = np.sqrt(T_balance/Ct)/2        #釣り合いの角速度
omega_break_zero = np.sqrt(T_balance/(Ct*3))
print("T_balance: ",T_balance)
print("omega_zero: ",omega_zero)
print("omega_break_zero: ",omega_break_zero)
#---------制御入力から入力電圧---------
kapa = CQ/Ct
A_val = 1/(2*T_balance*K)
print("A_val",A_val)

B_list = np.array([
                [-1, -1, -1, -1],
                [-x1, -x2, x3, x4],
                [y1, -y2, -y3, y4],
                [1, -1, 1, -1]])

B_inv = np.linalg.inv(B_list)

C_list = np.array([
                [1,0,0,0],
                [0,1,0,0],
                [0,0,1,0,],
                [0,0,0,kapa]])
C_inv = np.linalg.inv(C_list)
D_list = np.array([[deltaw],[deltaP],[deltaQ],[deltaR]])
e_list = A_val * B_inv @ C_inv @ D_list

#---------釣り合いの電圧---------
RK = Reg/K
e0_alt = 0
e0 = (K*omega_zero)+(CQ*omega_zero**2*RK)
# e0_break = 11.1
print("e0: ",e0)
e1 = e0 + e_list[0][0]
e2 = e0 + e_list[1][0]
e3 = e0 + e_list[2][0]
e4 = e0 + e_list[3][0]

#---------ガウスノイズ---------
mean = 0

std_dev_uvw = 0.001
std_dev_p = 0.01
std_dev_q = 0.01
std_dev_r = 0.01
std_dev_phi = 0.0
std_dev_theta = 0.0
std_dev_psi = 0.0
std_dev_w = 0.0
std_dev_z = 0.0
std_dev_omega = 0.01

# std_dev_uvw = 0.001
# std_dev_p = 0.005
# std_dev_q = 0.005
# std_dev_r = 0.003
# std_dev_phi = 0.007
# std_dev_theta = 0.007
# std_dev_psi = 0.0007
# std_dev_w = 0.0
# std_dev_z = 0.0
# std_dev_omega = 0.01

#---------描画用配列---------
U,V,W = [],[],[]
P,Q,R = [],[],[]
PHI,THETA,PSI = [],[],[]
X,Y,Z = [],[],[]
BX,BY = [],[]
T_ref_list = []
T_list = []
Delta_list = []
omega_fr_list,omega_fl_list,omega_rl_list,omega_rr_list = [],[],[],[]
Ref_p_list,Ref_q_list,Ref_r_list,Ref_w_list = [],[],[],[]
deltaP_list,deltaQ_list,deltaR_list,deltaw_list = [],[],[],[]
deltaP_B,deltaQ_B = [],[]
break_t_list = []
Tfr_list,Tfl_list,Trr_list,Trl_list=[],[],[],[]
e1_list,e2_list,e3_list,e4_list = [],[],[],[]
#---------シミュレーション設定---------
SMAX = 30000.0
h = 0.01
sim_h = 0.0001
dt = h
t = 0.0 
T =1
time_steps = int(T / sim_h)
alt_flag = 0
break_flag = 0
last_k = 0
k=0
psi_ang=0
last_psi_ang = 0
ang_ref_flag = 0
ang_ref_count = 0
break_pid_count_1 = 0
break_pid_count_1_max = h*5
break_pid_count_2 = 0
break_pid_count_2_max = h*5
break_pid_flag = 0
break_deltaPQ=0
break_t = 0

std_dev_p = 0.1
std_dev_q = 0.1
std_dev_r = 0.05
std_dev_phi = 0.01
std_dev_theta = 0.01
std_dev_psi = 0.005

#---------MPC---------
nx = 6  # 状態次元（φ, θ, ψ, p, q, r）
nu = 3  # 入力次元（δP, δQ, δR）
N = 20  # 予測ホライゾン

# === 状態方程式のA行列とB行列 ===
A = np.array([
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0],
    [0, 0, 0, 0, 0, 0]
])

numerator = l * Ct * K * omega_zero * 111.112 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Ix
Bp = numerator / denominator

numerator = l * Ct * K * omega_zero * 111.112 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg + K**2)**2 * Iy
Bq = numerator / denominator

numerator = 2 * CQ * K * omega_zero * 27.667 * (K**2 + CQ * omega_zero * Reg)
denominator = (D * Reg+ K**2)**2 * Iz
Br = numerator / denominator

print(Bp, Bq, Br)
print(1/Ix, 1/Iy, 1/Iz)


B = np.array([
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0],
    [Bp, 0, 0],
    [0, Bq, 0],  
    [0, 0, Br]])


# B = np.array([
#     [0, 0, 0],
#     [0, 0, 0],
#     [0, 0, 0],
#     [1/Ix, 0, 0],
#     [0, 1/Iy, 0],
#     [0, 0, 1/Iz]
# ])

# === 指数関数法での離散化 ===
# A_d, B_d = discretize_matrix_exponential(A, B, dt)
A_d, B_d = discretize_euler(A, B, dt)  # euler法での離散化も可能

# === NMPCの非線形状態遷移モデル ===
def drone_nonlinear_dynamics_casadi():
    x = ca.MX.sym('x', nx)
    u = ca.MX.sym('u', nu)

    phi, theta, psi, p, q, r = x[0], x[1], x[2], x[3], x[4], x[5]
    dP, dQ, dR = u[0], u[1], u[2]

    phi_dot = p + q * ca.sin(phi) * ca.tan(theta) + r * ca.cos(phi) * ca.tan(theta)
    theta_dot = q * ca.cos(phi) - r * ca.sin(phi)
    psi_dot = (q * ca.sin(phi) + r * ca.cos(phi)) / ca.cos(theta)

    p_dot = (dP - (Iz - Iy) * q * r - ca.sign(p) * Cp * p**2) / Ix
    q_dot = (dQ - (Ix - Iz) * r * p - ca.sign(q) * Cq * q**2) / Iy
    r_dot = (dR - (Iy - Ix) * p * q - ca.sign(r) * Cr * r**2) / Iz

    x_dot = ca.vertcat(phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot)
    x_next = x + dt * x_dot
    return ca.Function('f', [x, u], [x_next])

u_min = np.tile(np.array([[-0.1], [-0.1], [-0.1]]), (N, 1))  # (nu*N, 1)
u_max = np.tile(np.array([[0.1], [0.1], [0.1]]), (N, 1))     # (nu*N, 1)


# === MPC制御関数 ===
def mpc_control(x0, x_ref):
    f = drone_nonlinear_dynamics_casadi()

    X = ca.MX.sym('X', nx, N+1)
    U = ca.MX.sym('U', nu, N)
    P = ca.MX.sym('P', 2*nx)

    Q = np.diag([1, 1, 0.1, 0.01, 0.01, 0.01])
    R = np.diag([10, 10, 10])

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
        'ubg': np.zeros(nx*(N+1)),
        'lbx': np.vstack([
        -np.inf * np.ones((nx*(N+1), 1)),  # 状態Xの下限は制限しない
        u_min                       # 入力Uの下限
        ]),
        'ubx': np.vstack([
            np.inf * np.ones((nx*(N+1), 1)),  # 状態Xの上限も制限しない
            u_max                       # 入力Uの上限
        ])
    }

    sol = solver(**args)
    U_opt = sol['x'][-nu*N:]
    u0 = np.array(U_opt[:nu]).flatten()
    return u0


#---------pid---------

#---------PIDクラスの宣言---------
class PIDController:
    def __init__(self, kp, ti, td):
        self.kp = kp
        self.ti = ti
        self.td = td
        self.s = 0.0
        self.err = 0.0
        

# ---------PIDインスタンスの生成---------
pid_w = PIDController(1, 1, 0.0)
pid_z = PIDController(1, 1, 0.0)
#---------PID関数宣言---------
def pid_W(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_w.s += err * h
    # 制限をかける
    if pid_w.s > SMAX:
        pid_w.s = SMAX
    elif pid_w.s < -SMAX:
        pid_w.s = -SMAX
    # 差分の計算
    diff = (err - pid_w.err) / h
    pid_w.err = err
    # PID制御計算式
    return pid_w.kp * (err + diff * pid_w.td + pid_w.s / pid_w.ti)

def pid_Z(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_z.s += err * h
    # 制限をかける
    if pid_z.s > SMAX:
        pid_z.s = SMAX
    elif pid_z.s < -SMAX:
        pid_z.s = -SMAX
    # 差分の計算
    diff = (err - pid_z.err) / h
    pid_z.err = err
    # PID制御計算式
    return pid_z.kp * (err + diff * pid_z.td + pid_z.s / pid_z.ti)
#---------PIDパラメータ設定---------
def reset_pid():
    pid_w.kp, pid_w.ti, pid_w.td, pid_w.s = 1, 1000, 0.01, 0.0   #1, 1000, 0.1, 0.0
    pid_z.kp, pid_z.ti, pid_z.td, pid_z.s = 1, 1000, 0.01, 0.0   #1, 1000, 0.5, 0.0

#---------正規化関数(min-maxスケーリング)---------
def normalize(values):
    min_val = min(values)
    max_val = max(values)
    # 正規化の公式 (value - min) / (max - min)
    normalized = [(x - min_val) / (max_val - min_val) if max_val != min_val else 0.0 for x in values]
    return normalized

def min_max(x, axis=None):
    min = x.min(axis=axis, keepdims=True)
    max = x.max(axis=axis, keepdims=True)
    result = (x-min)/(max-min)
    return result

#---------4次ルンゲクッタシュミレーション---------
def rk4(func,y,t,h):
    k1 = func(t,y)
    k2 = func(t+h/2 , y+k1*h/2)
    k3 = func(t+h/2 , y+k2*h/2)
    k4 = func(t+h , y+k3*h)
    y = y + (h/6)*(k1 + 2*k2 + 2*k3 + k4)
    return y
#---------三角関数---------
def cos(ang):
    return np.cos(ang)

def sin(ang):
    return np.sin(ang)

def tan(ang):
    return np.tan(ang)

#---------数値計算関数定義---------
#---------加速度(オイラー角)---------
def u_dot(t, u):
    u_sign = math.copysign(1, u)
    return (-g * math.sin(theta)) - (u_sign * (Cu * u**2) / m )- (q * w) + (r * v)

def v_dot(t, v):
    v_sign = math.copysign(1, v)
    return (g * cos(theta) * sin(phi)) - (v_sign * (Cv * v**2) / m) - (r * u) + (q * w)

def w_dot(t, w):
    w_sign = math.copysign(1, w)
    return (g * cos(theta) * cos(phi)) - (w_sign * (Cw * w**2) / m) - (Tfr/m) - (Tfl/m) - (Trr/m) - (Trl/m) - (q * v) + (q * u)

#---------加速度(クォータ二オン)---------
# def u_dot(t, u):
#     u_sign = math.copysign(1, u)
#     return (-g * 2*(q1*q3 - q0*q2)) - (u_sign * (Cu * u**2) / m )- (q * w) + (r * v)

# def v_dot(t, v):
#     v_sign = math.copysign(1, v)
#     return (g * 2*(q2*q3 + q0*q1)) - (v_sign * (Cv * v**2) / m) - (r * u) + (q * w)

# def w_dot(t, w):
#     w_sign = math.copysign(1, w)
#     return (g * (q0**2 - q1**2 - q2**2 + q3**2)) - (w_sign * (Cw * w**2) / m) - (Tfr/m) - (Tfl/m) - (Trr/m) - (Trl/m) - (q * v) + (q * u)


#---------モーターの角加速度---------
def omega_dot_fr(t,omega_fr):
    return ((K * e1) / (Reg * Iz_m)) - (Qf / Iz_m) - ((CQ * omega_fr**2)/Iz_m) - (((D + (K**2)) / Reg) * omega_fr / Iz_m)

def omega_dot_fl(t,omega_fl):
    return ((K * e4) / (Reg * Iz_m)) - (Qf / Iz_m) - ((CQ * omega_fl**2)/Iz_m) - (((D + (K**2)) / Reg) * omega_fl / Iz_m)

def omega_dot_rr(t,omega_rr):
    return ((K * e2) / (Reg * Iz_m)) - (Qf / Iz_m) - ((CQ * omega_rr**2)/Iz_m) - (((D + (K**2)) / Reg) * omega_rr / Iz_m)

def omega_dot_rl(t,omega_rl):
    return ((K * e3) / (Reg * Iz_m)) - (Qf / Iz_m) - ((CQ * omega_rl**2)/Iz_m) - (((D + (K**2)) / Reg) * omega_rl / Iz_m)

#---------角加速度---------
def p_dot(t,p):
    p_sign = math.copysign(1, p)
    return ((0.5 * l * (Tfl + Trl - Tfr - Trr)) / Ix) - (((Iz - Iy) * q * r) / Ix) - (p_sign * (Cp * p**2) / Ix)

def q_dot(t,q):
    q_sign = math.copysign(1, q)
    return ((0.5 * l * (Tfr + Tfl - Trr - Trl))) / Iy - (((Ix - Iz) * r * p) / Iy) - (q_sign * (Cq * q**2) / Iy)

def r_dot(t,r):
    r_sign = math.copysign(1, r)
    return (((Qfr + Qrl - Qfl - Qrr)) / Iz) - (((Iy - Ix) * p * q) / Iz) - (r_sign * (Cr * r**2) / Iz)

#---------角速度(オイラー角)---------
def phi_dot(t,phi):
    return p + (q * sin(phi) * tan(theta)) + (r * cos(phi) * tan(theta))

def theta_dot(t,theta):
    return (q * cos(phi)) - (r * sin(phi))

def psi_dot(t,psi):
    return ((q * sin(phi)) + (r * cos(phi))) / cos(theta)
#---------角速度(クォータ二オン)---------
def q0_dot(t,q0):
    return 0.5*((-p*q1) -(q*q2) + (r*q3))

def q1_dot(t,q1):
    return 0.5*((p*q0) + (r*q2) - (q*q3))

def q2_dot(t,q2):
    return 0.5*((q*q0) - (r*q1) + (p*q3))

def q3_dot(t,q3):
    return 0.5*((r*q0) + (q*q1) - (r*q2))

#---------速度の計算(オイラー角)---------
def Xe_dot(t,Xe):
    return (u * cos(theta) * cos(psi)) +(v * ((sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi)))) + (w * ((cos(phi) * sin(theta)) * (cos(psi) + sin(phi) * sin(psi))))

def Ye_dot(t,Ye):
    return (u * cos(theta) * sin(psi)) + (v * ((sin(phi) * sin(theta) * sin(psi)) + (cos(phi) * cos(psi)))) + (w * ((cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))))

def Ze_dot(t,Ze):
    return (-u * sin(theta)) + (v * sin(phi) * cos(theta)) + (w * cos(phi) * cos(theta))

#---------速度の計算(クォータ二オン角)---------
# def Xe_dot(t,Xe):
#     return (u*(q0**2 + q1**2 -q2**2 -q3**2) + 2*v*(q1*q2 + q0*q3) + 2*w*(q1*q3 - q0*q2))

# def Ye_dot(t,Ye):
#     return ((2*u*(q1*q2-q0*q3) + v*(q0**2 - q1**2 + q2**2 -q3**2)) + 2*w*(q2*q3 + q0*q1))

# def Ze_dot(t,Ze):
#     return (2*u*(q1*q3 + q0*q2) + 2*v*(q2*q3 - q0*q1) + w*(q0**2 - q1**2 -q2**2 + q3**2))

#---------PIDの初期化---------
reset_pid()
# === テスト ===
x0 = np.array([0, 0, 0, 0, 0, 0])  # 初期状態（φ, θ, ψ, p, q, r） 
x_ref = np.array([0, 0, 0, 0, 0, 0])  # 目標状態
# phi = 0
mpc_u = mpc_control(x0, x_ref)
print("最適化された初期入力:", mpc_u)
mpc_count = 0
alt_count = 0
mpc_c = 0
alt_c = 0
#---------メインループ開始---------
for i in range(time_steps):
    t += h
    # データの保存
    X.append(Xe)
    Y.append(Ye)
    Z.append(-1*Ze)
    # Z.append(Ze)
    U.append(u)
    V.append(v)
    W.append(-1*w)
    # W.append(w)
    P.append(p)
    Q.append(q)
    R.append(r)
    PHI.append(phi)
    THETA.append(theta)
    PSI.append(psi)
    T_list.append(i*sim_h)
    T_ref_list.append(omega_dot_fl(t,omega_fl))
    # Delta_list.append(-1*Ref_w)
    omega_fr_list.append(omega_fr)
    omega_fl_list.append(omega_fl)
    omega_rr_list.append(omega_rr)
    omega_rl_list.append(omega_rl)
    deltaP_list.append(mpc_u[0])
    deltaQ_list.append(mpc_u[1])
    deltaR_list.append(mpc_u[2])
    deltaw_list.append(deltaw)
    Ref_w_list.append(Ref_w)
    e1_list.append(e1)
    e2_list.append(e2)
    e3_list.append(e3)
    e4_list.append(e4)

    if(i>(3/h)):
        x_ref = np.array([1, 0, 0, 0, 0, 0])  # 目標状態を更新（例としてゼロに設定）

    # === MPCで制御入力を計算 ===
    mpc_count += 1
    alt_count += 1
    if mpc_count >= 25:
        mpc_count = 0
        mpc_c += 1
        # === MPCで制御入力を計算 ===
        mpc_u = mpc_control(x0, x_ref)  # ← ここで最適化
    # mpc_u = mpc_control(x0, x_ref)  # ← ここで最適化

    # #---------高さPID---------
    # if(break_flag == 0):

    if alt_count >= 25:
        alt_count = 0
        alt_c += 1
        Ref_z = -2.0
        if(-1*Ze < -1*Ref_z - 0.3 and alt_flag == 0):
        # Ref_z = 2.0
        # if(Ze < Ref_z - 0.3 and alt_flag == 0):
            e0_alt = e0+1
            alt_flag = 0
        else:
            alt_flag = 1

        if alt_flag == 1:
            #ノイズ有り
            # Ref_w = pid_Z(Ref_z,Ze + np.random.normal(mean, std_dev_z),1)
            # deltaw = pid_W(Ref_w,w + np.random.normal(mean, std_dev_w),1)
            #ノイズ無し
            Ref_w = pid_Z(Ref_z,Ze,1)
            deltaw = pid_W(Ref_w,w,1)

    #---------制御入力から入力電圧---------
    #最初高度上げる時
    if(alt_flag == 0):
        D_list = np.array([[deltaw],[mpc_u[0]],[mpc_u[1]],[mpc_u[2]]])
        e_list = A_val * B_inv @ C_inv @ D_list
        e1 = e0_alt + (e_list[0][0])
        e2 = e0_alt + (e_list[1][0])
        e3 = e0_alt + (e_list[2][0])
        e4 = e0_alt + (e_list[3][0])
    #通常時
    else:
        D_list = np.array([[deltaw],[mpc_u[0]],[mpc_u[1]],[mpc_u[2]]])
        e_list = A_val * B_inv @ C_inv @ D_list
        e1 = e0 + (e_list[0][0])
        e2 = e0 + (e_list[1][0])
        e3 = e0 + (e_list[2][0])
        e4 = e0 + (e_list[3][0])

    # if(e1 >= 11.1):
    #     e1 = 11.1
    # if(e2 >= 11.1):
    #     e2 = 11.1
    # if(e3 >= 11.1):
    #     e3 = 11.1
    # if(e4 >= 11.1):
    #     e4 = 11.1

    if(e1 >= 12):
        e1 = 12
    if(e2 >= 12):
        e2 = 12
    if(e3 >= 12):
        e3 = 12
    if(e4 >= 12):
        e4 = 12
    
    if(e1 <= 0):
        e1 = 0
    if(e2 <= 0):
        e2 = 0
    if(e3 <= 0):
        e3 = 0
    if(e4 <= 0):
        e4 = 0

    #---------モーターの角速度---------
    omega_fr = rk4(omega_dot_fr, omega_fr, t, h)
    omega_fl = rk4(omega_dot_fl, omega_fl, t, h)
    omega_rr = rk4(omega_dot_rr, omega_rr, t, h)
    omega_rl = rk4(omega_dot_rl, omega_rl, t, h)

    if(omega_fr<0): omega_fr = 0
    if(omega_fl<0): omega_fl = 0
    if(omega_rr<0): omega_rr = 0
    if(omega_rl<0): omega_rl = 0

    #---------トルク計算---------
    Qfr = CQ * omega_fr**2
    Qfl = CQ * omega_fl**2
    Qrr = CQ * omega_rr**2
    Qrl = CQ * omega_rl**2
    #---------推力計算---------
    Tfr = Ct * omega_fr**2
    Tfl = Ct * omega_fl**2
    Trr = Ct * omega_rr**2
    Trl = Ct * omega_rl**2

    #---------速度の計算---------
    u = rk4(u_dot, u, t, h) 
    v = rk4(v_dot, v, t, h) 
    w = rk4(w_dot, w, t, h) 

    #---------角速度の計算---------
    p = rk4(p_dot, p, t, h)
    q = rk4(q_dot, q, t, h)
    r = rk4(r_dot, r, t, h) 
    #---------角度の計算---------
    phi =  rk4(phi_dot, phi, t, h)
    theta = rk4(theta_dot, theta, t, h)
    psi = rk4(psi_dot, psi, t, h)

    # E11 = q0**2 + q1**2 - q2**2 - q3**2
    # E12 = 2*(q1*q2 + q0*q3)
    # E13 = 2*(q1*q3 - q0*q2)
    # E21 = 2*(q1*q2 - q0*q3)
    # E22 = q0**2 - q1**2 + q2**2 - q3**2
    # E23 = 2*(q2*q3 + q0*q1)
    # E31 = 2*(q1*q3 + q0*q2)
    # E32 = 2*(q2*q3 - q0*q1)
    # E33 = q0**2 - q1**2 - q2**2 + q3**2

    # phi = math.atan2(2*(q2*q3 + q0*q1),q0**2 - q1**2 - q2**2 + q3**2)
    # theta = math.atan2(-2*(q1*q3 - q0*q2),np.sqrt((2*(q2*q3 + q0*q1))**2 + (q0**2 - q1**2 - q2**2 + q3**2)**2))
    # # psi = math.atan2(2*(q1*q2 + q0*q3),q0**2 + q1**2 - q2**2 - q3**2)

    # q0 = rk4(q0_dot,q0,t,h) 
    # q1 = rk4(q1_dot,q1,t,h) 
    # q2 = rk4(q2_dot,q2,t,h) 
    # q3 = rk4(q3_dot,q3,t,h) 

    # norm_num = np.sqrt(q0**2 + q1**2 + q2**2 + q3**2)
    
    # q0 = q0 / norm_num
    # q1 = q1 / norm_num
    # q2 = q2 / norm_num
    # q3 = q3 / norm_num

    #math.atanはpi/2から-pi/2  math.atan2はpiから-pi
    # phi = math.atan2(E23,E33)
    # theta = math.atan2(-E13,np.sqrt(E23**2 + E33**2))
    # psi = math.atan2(E12,E11)

    #---------位置の計算---------
    Xe = rk4(Xe_dot, Xe, t, h)
    Ye = rk4(Ye_dot, Ye, t, h)
    Ze = rk4(Ze_dot, Ze, t, h) 

   #x0 = np.array([phi, theta, psi, p, q, r])  # 状態を更新
    x0 = np.array([ phi+ np.random.normal(mean, std_dev_phi),
                theta+ np.random.normal(mean, std_dev_theta),
                psi+ np.random.normal(mean, std_dev_psi), 
                p+ np.random.normal(mean, std_dev_p), 
                q+ np.random.normal(mean, std_dev_q), 
                r+ np.random.normal(mean, std_dev_r)])  # 状態を更新
    #---------デバッグ用プリント---------

end_time = time.time()  # 終了時間
elapsed_time = end_time - start_time  # 経過時間（秒）

print(f"実行時間: {elapsed_time:.4f} 秒")

# 時間軸データ (ステップごと)
time = T_list 
print(time_steps)
print("姿勢制御の周期[Hz]",mpc_c/T)
print("高度制御の周期[Hz]",alt_c/T)
#---------デバッグ用グラフ---------
# # 図のサイズを設定
fig, axs = plt.subplots(4, 4, figsize=(20, 15))
fig.suptitle("Simulation Data Visualization")

# 位置のプロット
axs[0, 0].plot(time, X, label='X')
# axs[0, 0].set_title("Position X [m]")
axs[0, 0].set_xlabel("Time Step")
axs[0, 0].set_ylabel("X Position")

axs[0, 1].plot(time, Y, label='Y')
# axs[0, 1].set_title("Position Y [m]")
axs[0, 1].set_ylabel("Y Position")
axs[0, 1].set_xlabel("Time Step")

axs[0, 2].plot(time, Z, label='Z')
# axs[0, 2].set_title("Position Z [m]")
axs[0, 2].set_xlabel("Time Step")
axs[0, 2].set_ylabel("Z Position")

# 速度のプロット
axs[1, 0].plot(time, U, label='U', color='r')
# axs[1, 0].set_title("Velocity U [m/s]")
axs[1, 0].set_xlabel("Time Step")
axs[1, 0].set_ylabel("U Velocity")

axs[1, 1].plot(time, V, label='V', color='g')
# axs[1, 1].set_title("Velocity V [m/s]")
axs[1, 1].set_xlabel("Time Step")
axs[1, 1].set_ylabel("V Velocity")

axs[1, 2].plot(time, W, label='W', color='b')
axs[1, 2].set_xlabel("Time Step")
axs[1, 2].set_ylabel("W Velocity")

# 角速度のプロット
axs[2, 0].plot(time, P, label='P', color='purple')
axs[2, 0].set_xlabel("Time Step")
axs[2, 0].set_ylabel("P Angular Velocity")

axs[2, 1].plot(time, Q, label='Q', color='orange')
axs[2, 1].set_xlabel("Time Step")
axs[2, 1].set_ylabel("Q Angular Velocity")

axs[2, 2].plot(time, R, label='R', color='brown')
axs[2, 2].set_xlabel("Time Step")
axs[2, 2].set_ylabel("R Angular Velocity")

# オイラー角のプロット
axs[0, 3].plot(time, PHI, label='Phi', color='cyan')
axs[0, 3].set_xlabel("Time Step")
axs[0, 3].set_ylabel("Phi Angle")

axs[1, 3].plot(time, THETA, label='Theta', color='magenta')
axs[1, 3].set_xlabel("Time Step")
axs[1, 3].set_ylabel("Theta Angle")

axs[2, 3].plot(time, PSI, label='Psi', color='black')
axs[2, 3].set_xlabel("Time Step")
axs[2, 3].set_ylabel("Psi Angle")

axs[3,0].plot(time,omega_fl_list,label='omega_fl')
axs[3,0].plot(time,omega_fr_list,label='omega_fr')
axs[3,0].plot(time,omega_rl_list,label='omega_rl')
axs[3,0].plot(time,omega_rr_list,label='omega_rr')
# axs[3,0].set_title("motor ang vel")
axs[2, 3].set_xlabel("Time Step")
axs[3, 0].set_ylabel("propeller omega [rad/s]")

axs[3,1].plot(time,deltaw_list,label='deltaw')
axs[3,1].plot(time,Ref_w_list,label='Ref_w')
axs[3,1].set_xlabel("Time Step")
axs[3,1].set_ylabel("altitude PID")

axs[3,2].plot(time,deltaP_list,label='deltaP')
axs[3,2].plot(time,deltaQ_list,label='deltaQ')
axs[3,2].plot(time,deltaR_list,label='deltaR')
# axs[3,2].plot(time,deltaw_list,label='deltaw')
axs[3,2].set_xlabel("Time Step")
axs[3,2].set_ylabel("control input")

axs[3,3].plot(time,e1_list,label='e_Fr')
axs[3,3].plot(time,e2_list,label='e_Rr')
axs[3,3].plot(time,e3_list,label='e_Rl')
axs[3,3].plot(time,e4_list,label='e_Fl')
axs[3, 3].set_xlabel("Time Step")
axs[3,3].set_ylabel("voltage")

#グリッドを追加
for ax in axs.flat:
    ax.grid(True)
    ax.legend()

# レイアウトの調整
plt.tight_layout(rect=[0, 0, 1, 0.95])
# グラフを表示
plt.show()