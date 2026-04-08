# pid_pd_response_compare.py
# P・PD・PI・PID制御のステップ応答を比較して描画するサンプル
# 一次遅れ系 (dy/dt = -(1/τ) y + (K/τ) u) をオイラー法で離散化

import numpy as np
import matplotlib.pyplot as plt

# ===== パラメータ設定 =====
setpoint = 1.0          # 目標値
t_end = 10           # シミュレーション時間 [s]
dt = 0.01             # サンプリング周期 [s]

# プラント（一次遅れ系）のパラメータ
tau = 1.0               # 時定数
K_plant = 1.0           # ゲイン

# 各制御器パラメータ
# 例: 明確にオーバーシュートを出す設定
params_P   = dict(Kp=1.0, Ki=0.0, Kd=0.0)      # ←Pは基本オーバーシュート出ない
params_PD  = dict(Kp=1.0, Ki=0.0, Kd=0.1)      # ←PDでも出にくい（一次ではほぼ出ない）
params_PI  = dict(Kp=1.0, Ki=1.0, Kd=0.0)      # ★オーバーシュートが出やすい
params_PID = dict(Kp=1.0, Ki=1.0, Kd=0.1)     # ★PIより少し抑えるが、それでも出やすい

# センサノイズ（微分項の影響確認用）
sensor_noise_std = 0.0
# =========================


# 置き換え用：2次プラントのパラメータ
wn = 2.0     # 自然周波数
zeta = 0.3   # 減衰比（小さいほどオーバーシュート大）

def simulate_pid_response(Kp: float, Ki: float, Kd: float):
    n = int(t_end / dt) + 1
    t = np.linspace(0, t_end, n)
    y = np.zeros(n)
    v = np.zeros(n)  # y'（速度）
    u = np.zeros(n)
    e_int = 0.0
    e_prev = 0.0

    for k in range(1, n):
        y_meas = y[k-1]  # ノイズ入れるなら + np.random.randn()*sensor_noise_std
        e = setpoint - y_meas
        e_int += e * dt
        e_der = (e - e_prev) / dt

        u[k] = Kp * e + Ki * e_int + Kd * e_der

        # 2次系をオイラー法で前進更新
        a = -2*zeta*wn*v[k-1] - (wn**2)*y[k-1] + (wn**2)*u[k]  # y'' = a
        v[k] = v[k-1] + a*dt
        y[k] = y[k-1] + v[k]*dt

        e_prev = e

    return t, y

def main():
    # 各制御方式をシミュレーション
    t, y_P   = simulate_pid_response(**params_P)
    _, y_PD  = simulate_pid_response(**params_PD)
    _, y_PI  = simulate_pid_response(**params_PI)
    _, y_PID = simulate_pid_response(**params_PID)

    # 描画
    plt.figure(figsize=(9, 5))
    plt.plot(t, y_P,   label="P")
    plt.plot(t, y_PD,  label="PD")
    plt.plot(t, y_PI,  label="PI")
    plt.plot(t, y_PID, label="PID")
    plt.axhline(setpoint, linestyle="--", color="gray", label="ref")
    plt.title("")
    plt.xlabel("[s]")
    plt.ylabel("y(t)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
