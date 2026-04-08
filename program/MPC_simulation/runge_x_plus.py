import numpy as np
import threading
import time
import math

#---------パラメータ設定---------
g = 9.81                                    # 重力加速度(m/s)
m = 0.71                                    # 機体質量(kg)
l = 0.18                                    #モーター間の距離(m)
# l = 0.254558                              #モーター間の距離(m)
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
# D_list = np.array([[-1*deltaw],[deltaP],[deltaQ],[deltaR]])
D_list = np.array([[deltaw],[deltaP],[deltaQ],[deltaR]])
e_list = A_val * B_inv @ C_inv @ D_list
#0度

B_break_list_0 = np.array([
                [-1, -1, -1],
                [-x2, 0, x4],
                [-y2, -y3, y4]])

B_break_inv_0 = np.linalg.inv(B_break_list_0)
#45度
B_break_list_45 = np.array([
                [1, 1, 1],
                [0,x3,0],
                [-y2, 0, y4]])
B_break_inv_45 = np.linalg.inv(B_break_list_45)
#90度
B_break_list_90 = np.array([
                [1, 1, 1],
                [-x1, x3, x4],
                [y1, -y3, y4]])
B_break_inv_90 = np.linalg.inv(B_break_list_90)
#180度
B_break_list_180 = np.array([
                [1, 1, 1],
                [-x1, -x2, x4],
                [y1, -y2, y4]])
B_break_inv_180 = np.linalg.inv(B_break_list_180)
#270度
B_break_list_270 = np.array([
                [1, 1, 1],
                [-x1,-x2, x3],
                [y1, -y2, -y3]])
B_break_inv_270 = np.linalg.inv(B_break_list_270)

C_break_list = np.array([
                [1,0,0],
                [0,1,0],
                [0,0,kapa]])
C_break_inv = np.linalg.inv(C_break_list)

D_break_list = np.array([[deltaw],[deltaP],[deltaQ]])
e_break_list = A_val * B_break_inv_0 @ C_break_inv @ D_break_list
e_break_list_0 = A_val * B_break_inv_0 @ C_break_inv @ D_break_list
e_break_list_90 = A_val * B_break_inv_90 @ C_break_inv @ D_break_list
e_break_list_180 = A_val * B_break_inv_180 @ C_break_inv @ D_break_list
e_break_list_270 = A_val * B_break_inv_270 @ C_break_inv @ D_break_list

#---------釣り合いの電圧---------
RK = Reg/K
e0_alt = 0
e0 = (K*omega_zero)+(CQ*omega_zero**2*RK)
e0_break = (K*omega_break_zero)+(CQ*omega_break_zero**2*RK)
e0_break_2 = 7.0159
e0_break_3 = 8.5559
e0_break_4 = 10.0959
# e0_break = 11.1
print("e0: ",e0)
print("e0_break: ",e0_break)
e1 = e0 + e_list[0][0]
e2 = e0 + e_list[1][0]
e3 = e0 + e_list[2][0]
e4 = e0 + e_list[3][0]

#---------ガウスノイズ---------
mean = 0

std_dev_uvw = 0.001
std_dev_p = 0.01
std_dev_q = 0.01
std_dev_r = 0.003
std_dev_phi = 0.007
std_dev_theta = 0.007
std_dev_psi = 0.0007
std_dev_w = 0.0
std_dev_z = 0.0
std_dev_omega = 0.01

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
dt = h #0.001
t = 0.0 
T =200
time_steps = int(T / dt)
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
pid_p = PIDController(1, 1, 0.0)
pid_q = PIDController(1, 1, 0.0)
pid_r = PIDController(1, 1, 0.0)
pid_phi = PIDController(1, 1, 0.0)
pid_theta = PIDController(1, 1, 0.0)
pid_psi = PIDController(1, 1, 0.0)
pid_w = PIDController(1, 1, 0.0)
pid_z = PIDController(1, 1, 0.0)

pid_p_break = PIDController(1, 1, 0.0)
pid_q_break = PIDController(1, 1, 0.0)
pid_phi_break = PIDController(1, 1, 0.0)
pid_theta_break = PIDController(1, 1, 0.0)
pid_w_break = PIDController(1, 1, 0.0)
pid_z_break = PIDController(1, 1, 0.0)

#---------PID関数宣言---------
def pid_P(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_p.s += err * h
    # 制限をかける
    if pid_p.s > SMAX:
        pid_p.s = SMAX
    elif pid_p.s < -SMAX:
        pid_p.s = -SMAX
    # 差分の計算
    diff = (err - pid_p.err) / h
    pid_p.err = err
    # PID制御計算式
    return pid_p.kp * (err + diff * pid_p.td + pid_p.s / pid_p.ti)

def pid_P_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_p_break.s += err * h
    # 制限をかける
    if pid_p_break.s > SMAX:
        pid_p_break.s = SMAX
    elif pid_p_break.s < -SMAX:
        pid_p_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_p_break.err) / h
    pid_p_break.err = err
    # PID制御計算式
    return pid_p_break.kp * (err + diff * pid_p_break.td + pid_p_break.s / pid_p_break.ti)

def pid_Q(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_q.s += err * h
    # 制限をかける
    if pid_q.s > SMAX:
        pid_q.s = SMAX
    elif pid_q.s < -SMAX:
        pid_q.s = -SMAX
    # 差分の計算
    diff = (err - pid_q.err) / h
    pid_q.err = err
    # PID制御計算式
    return pid_q.kp * (err + diff * pid_q.td + pid_q.s / pid_q.ti)

def pid_Q_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_q_break.s += err * h
    # 制限をかける
    if pid_q_break.s > SMAX:
        pid_q_break.s = SMAX
    elif pid_q_break.s < -SMAX:
        pid_q_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_q_break.err) / h
    pid_q_break.err = err
    # PID制御計算式
    return pid_q_break.kp * (err + diff * pid_q_break.td + pid_q_break.s / pid_q_break.ti)

def pid_R(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_r.s += err * h
    # 制限をかける
    if pid_r.s > SMAX:
        pid_r.s = SMAX
    elif pid_r.s < -SMAX:
        pid_r.s = -SMAX
    # 差分の計算
    diff = (err - pid_r.err) / h
    pid_r.err = err
    # PID制御計算式
    return pid_r.kp * (err + diff * pid_r.td + pid_r.s / pid_r.ti)

def pid_Phi(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_phi.s += err * h
    # 制限をかける
    if pid_phi.s > SMAX:
        pid_phi.s = SMAX
    elif pid_phi.s < -SMAX:
        pid_phi.s = -SMAX
    # 差分の計算
    diff = (err - pid_phi.err) / h
    pid_phi.err = err
    # PID制御計算式
    return pid_phi.kp * (err + diff * pid_phi.td + pid_phi.s / pid_phi.ti)

def pid_Phi_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_phi_break.s += err * h
    # 制限をかける
    if pid_phi_break.s > SMAX:
        pid_phi_break.s = SMAX
    elif pid_phi_break.s < -SMAX:
        pid_phi_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_phi_break.err) / h
    pid_phi_break.err = err
    # PID制御計算式
    return pid_phi_break.kp * (err + diff * pid_phi_break.td + pid_phi_break.s / pid_phi_break.ti)

def pid_Theta(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_theta.s += err * h
    # 制限をかける
    if pid_theta.s > SMAX:
        pid_theta.s = SMAX
    elif pid_theta.s < -SMAX:
        pid_theta.s = -SMAX
    # 差分の計算
    diff = (err - pid_theta.err) / h
    pid_theta.err = err
    # PID制御計算式
    return pid_theta.kp * (err + diff * pid_theta.td + pid_theta.s / pid_theta.ti)

def pid_Theta_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_theta_break.s += err * h
    # 制限をかける
    if pid_theta_break.s > SMAX:
        pid_theta_break.s = SMAX
    elif pid_theta_break.s < -SMAX:
        pid_theta_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_theta_break.err) / h
    pid_theta_break.err = err
    # PID制御計算式
    return pid_theta_break.kp * (err + diff * pid_theta_break.td + pid_theta_break.s / pid_theta_break.ti)

def pid_Psi(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_psi.s += err * h
    # 制限をかける
    if pid_psi.s > SMAX:
        pid_psi.s = SMAX
    elif pid_psi.s < -SMAX:
        pid_psi.s = -SMAX
    # 差分の計算
    diff = (err - pid_psi.err) / h
    pid_psi.err = err
    # PID制御計算式
    return pid_psi.kp * (err + diff * pid_psi.td + pid_psi.s / pid_psi.ti)

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

def pid_W_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_w_break.s += err * h
    # 制限をかける
    if pid_w_break.s > SMAX:
        pid_w_break.s = SMAX
    elif pid_w_break.s < -SMAX:
        pid_w_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_w_break.err) / h
    pid_w_break.err = err
    # PID制御計算式
    return pid_w_break.kp * (err + diff * pid_w_break.td + pid_w_break.s / pid_w_break.ti)

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

def pid_Z_break(ref, rate, flag):
    err = ref - rate
    if flag:
        pid_z_break.s += err * h
    # 制限をかける
    if pid_z_break.s > SMAX:
        pid_z_break.s = SMAX
    elif pid_z_break.s < -SMAX:
        pid_z_break.s = -SMAX
    # 差分の計算
    diff = (err - pid_z_break.err) / h
    pid_z_break.err = err
    # PID制御計算式
    return pid_z_break.kp * (err + diff * pid_z_break.td + pid_z_break.s / pid_z_break.ti)

#---------PIDパラメータ設定---------
def reset_pid():
    # 各コントローラのパラメータをリセット
    #X
    pid_p.kp, pid_p.ti, pid_p.td, pid_p.s = 0.004, 1, 0.01, 0.0
    pid_q.kp, pid_q.ti, pid_q.td, pid_q.s = 0.006, 0.5, 0.01, 0.0
    pid_r.kp, pid_r.ti, pid_r.td, pid_r.s = 0.06, 0.5, 0.01, 0.0 #100 ,1000
    pid_phi.kp, pid_phi.ti, pid_phi.td, pid_phi.s = 3, 1000, 0.01, 0.0
    pid_theta.kp, pid_theta.ti, pid_theta.td, pid_theta.s = 3, 1000, 0.01, 0.0
    pid_psi.kp, pid_psi.ti, pid_psi.td, pid_psi.s = 1, 1000, 0.01, 0.0
    #+
    # pid_p.kp, pid_p.ti, pid_p.td, pid_p.s = 0.001, 1, 0.01, 0.0
    # pid_q.kp, pid_q.ti, pid_q.td, pid_q.s = 0.015, 1, 0.01, 0.0
    # pid_r.kp, pid_r.ti, pid_r.td, pid_r.s = 0.04, 1, 0.01, 0.0 #100 ,1000
    # pid_phi.kp, pid_phi.ti, pid_phi.td, pid_phi.s = 1.5, 1000, 0.01, 0.0
    # pid_theta.kp, pid_theta.ti, pid_theta.td, pid_theta.s = 3, 1000, 0.01, 0.0
    # pid_psi.kp, pid_psi.ti, pid_psi.td, pid_psi.s = 1.5, 1000, 0.01, 0.0
    #e0 = 7.05
    pid_w.kp, pid_w.ti, pid_w.td, pid_w.s = 1, 1000, 0.01, 0.0   #1, 1000, 0.1, 0.0
    pid_z.kp, pid_z.ti, pid_z.td, pid_z.s = 1, 1000, 0.01, 0.0   #1, 1000, 0.5, 0.0

    pid_p_break.kp, pid_p_break.ti, pid_p_break.td, pid_p_break.s = 0.004, 1, 0.01, 0.0
    pid_q_break.kp, pid_q_break.ti, pid_q_break.td, pid_q_break.s = 0.006, 1, 0.01, 0.0
    pid_phi_break.kp, pid_phi_break.ti, pid_phi_break.td, pid_phi_break.s = 3, 1000, 0.01, 0.0
    pid_theta_break.kp, pid_theta_break.ti, pid_theta_break.td, pid_theta_break.s = 3, 1000, 0.01, 0.0
    pid_w_break.kp, pid_w_break.ti, pid_w_break.td, pid_w_break.s = 1, 500, 0.01, 0.0
    pid_z_break.kp, pid_z_break.ti, pid_z_break.td, pid_z_break.s = 1, 100, 0.01, 0.0

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
#---------ヨー軸回転数計算---------
def decompose_angle(psi):
    # psiを2πで割る
    # divmodを使うと、商と余りを同時に取得できる
    k, r = divmod(psi, 2*math.pi)
    
    # 商kは浮動小数点数になりうるので、整数部分にキャストしておく
    k = int(k)
    return k, r

def detect_special_angles(theta, tol=5):
    global last_k,k
    last_k = k
    k, r = decompose_angle(theta)
    r_deg = math.degrees(r)  # ラジアン→度数への変換
    # print(f"psi_deg:{r_deg}")
    if(k != last_k):
        print(f"psi_deg:{r_deg} {1}")
        # print(f"This angle is equivalent to 0° (k={k},last_k={last_k} r={r_deg}°)")
        return(1)
    elif math.isclose(r_deg, 1, abs_tol=tol):
        print(f"psi_deg:{r_deg} {2}")
        # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
        return(2)
    # 特定の角度に近いか判定
    # 0°, 90°, 180°, 270°を判定
    # if math.isclose(r_deg, 0, abs_tol=tol) or math.isclose(r_deg, 360, abs_tol=tol):
    #     print("This angle is equivalent to 0°")
    # elif math.isclose(r_deg, 40, abs_tol=tol):
    #     # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
    #     return(3)
    # elif math.isclose(r_deg, 50, abs_tol=tol):
    #     # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
    #     return(4)
    elif math.isclose(r_deg, 90, abs_tol=tol):
        print(f"psi_deg:{r_deg} {5}")
        # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
        return(5)
    # elif math.isclose(r_deg, 95, abs_tol=tol):
    #     print(f"psi_deg:{r_deg} {6}")
    #     # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
    #     return(6)
    # elif math.isclose(r_deg, 135, abs_tol=tol):
    #     # print(f"This angle is equivalent to 90° (k={k}, r={r_deg}°)")
    #     return(6)
    elif math.isclose(r_deg, 180, abs_tol=tol):
        print(f"psi_deg:{r_deg} {7}")
        # print(f"This angle is equivalent to 180° (k={k}, r={r_deg}°)")
        return(7)
    # elif math.isclose(r_deg, 185, abs_tol=tol):
    #     print(f"psi_deg:{r_deg} {8}")
    #     # print(f"This angle is equivalent to 180° (k={k}, r={r_deg}°)")
    #     return(8)

    elif math.isclose(r_deg, 270, abs_tol=tol):
        print(f"psi_deg:{r_deg} {9}")
        # print(f"This angle is equivalent to 270° (k={k}, r={r_deg}°)")
        return(9)
    # elif math.isclose(r_deg, 275, abs_tol=tol):
    #     print(f"psi_deg:{r_deg} {10}")
    #     # print(f"This angle is equivalent to 270° (k={k}, r={r_deg}°)")
    #     return(10)
    elif math.isclose(r_deg, 359, abs_tol=tol):
        print(f"psi_deg:{r_deg} {11}")
        # print(f"This angle is equivalent to 270° (k={k}, r={r_deg}°)")
        return(11)
    else:
        # print(f"This angle does not match 0°, 90°, 180°, or 270° exactly. (k={k}, r={r_deg}°)")
        return(0)
    
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
# def u_dot(t, u):
#     u_sign = math.copysign(1, u)
#     return (-g * math.sin(theta)) - (u_sign * (Cu * u**2) / m )- (q * w) + (r * v)

# def v_dot(t, v):
#     v_sign = math.copysign(1, v)
#     return (g * cos(theta) * sin(phi)) - (v_sign * (Cv * v**2) / m) - (r * u) + (q * w)

# def w_dot(t, w):
#     w_sign = math.copysign(1, w)
#     return (g * cos(theta) * cos(phi)) - (w_sign * (Cw * w**2) / m) - (Tfr/m) - (Tfl/m) - (Trr/m) - (Trl/m) - (q * v) + (q * u)

#---------加速度(クォータ二オン)---------
def u_dot(t, u):
    u_sign = math.copysign(1, u)
    return (-g * 2*(q1*q3 - q0*q2)) - (u_sign * (Cu * u**2) / m )- (q * w) + (r * v)

def v_dot(t, v):
    v_sign = math.copysign(1, v)
    return (g * 2*(q2*q3 + q0*q1)) - (v_sign * (Cv * v**2) / m) - (r * u) + (q * w)

def w_dot(t, w):
    w_sign = math.copysign(1, w)
    return (g * (q0**2 - q1**2 - q2**2 + q3**2)) - (w_sign * (Cw * w**2) / m) - (Tfr/m) - (Tfl/m) - (Trr/m) - (Trl/m) - (q * v) + (q * u)


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
# def Xe_dot(t,Xe):
#     return (u * cos(theta) * cos(psi)) +(v * ((sin(phi) * sin(theta) * cos(psi)) - (cos(phi) * sin(psi)))) + (w * ((cos(phi) * sin(theta)) * (cos(psi) + sin(phi) * sin(psi))))

# def Ye_dot(t,Ye):
#     return (u * cos(theta) * sin(psi)) + (v * ((sin(phi) * sin(theta) * sin(psi)) + (cos(phi) * cos(psi)))) + (w * ((cos(phi) * sin(theta) * sin(psi)) - (sin(phi) * cos(psi))))

# def Ze_dot(t,Ze):
#     return (-u * sin(theta)) + (v * sin(phi) * cos(theta)) + (w * cos(phi) * cos(theta))

#---------速度の計算(クォータ二オン角)---------
def Xe_dot(t,Xe):
    return (u*(q0**2 + q1**2 -q2**2 -q3**2) + 2*v*(q1*q2 + q0*q3) + 2*w*(q1*q3 - q0*q2))

def Ye_dot(t,Ye):
    return ((2*u*(q1*q2-q0*q3) + v*(q0**2 - q1**2 + q2**2 -q3**2)) + 2*w*(q2*q3 + q0*q1))

def Ze_dot(t,Ze):
    return (2*u*(q1*q3 + q0*q2) + 2*v*(q2*q3 - q0*q1) + w*(q0**2 - q1**2 -q2**2 + q3**2))

#---------PIDの初期化---------
reset_pid()