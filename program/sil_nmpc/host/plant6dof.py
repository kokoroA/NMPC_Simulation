"""ヘキサコプタの 6 自由度プラント（世界モデル）。2026-10-08。

旧 `plant.AttitudePlant` は「姿勢 6 状態を積分し、位置・速度は可視化用に開ループで足すだけ」だった。
ここでは並進と回転を**連成**させ、次を入れる（どれも個別に ON/OFF できる。全部 OFF で旧プラントに一致）。

| 要素 | 式 | 効く先 |
|---|---|---|
| 並進（機体座標） | m(u̇+qw−rv) = −mg sinθ + F_x,  m(v̇+ru−pw) = mg cosθ sinφ + F_y,  m(ẇ+pv−qu) = mg cosθ cosφ − T + F_z | 速度・位置 |
| 位置 | Ẋ_e = R(q)·[u,v,w]（クォータニオン表現。オイラー表現と一致することをテストで確認） | 位置 |
| 姿勢 | q̇ = ½ q⊗[0,p,q,r]（特異点なし） | 姿勢 |
| 回転 | I ω̇ = τ_motor − ω×(Iω)（ファームと同じ対角慣性形）− C ω|ω| + τ_aero + τ_gyro | 角速度 |
| モータ | 回転数の一次遅れ ω̇_i = (ω_ss(d_i) − ω_i)/τ（上げ・下げ別 τ 可）、T_i = Ct ω_i²、ω_ss = √(Tf(d)/Ct) | 推力・反トルク |
| ロータ加速の反トルク | τ_z += Σ kS_i J_r ω̇_i | ヨー |
| ジャイロ効果 | τ = −ω_body × (J_r Σ d_i ω_i e_z) | ロール・ピッチ |
| 二次抗力（形状抗力） | F = −C v_a|v_a|（各軸。v_a は対気速度） | 速度 |
| ロータ抗力（線形） | F_xy = −k_rd T v_a,xy | 速度 |
| ブローバック（速度安定） | L = −k_bf,roll v_a,  M = +k_bf,pitch u_a | 姿勢 ← 速度 |
| 地面効果 | T ×= 1/(1 − (R_p/4z)²)（Cheeseman–Bennett） | 推力 |
| 軸流による推力低下 | T ×= 1 − V_in/V_ps（V_in: 上昇でロータを下から抜ける流速） | 推力 |
| 風 | 一定風 + 一次ガウス・マルコフの突風 | 対気速度 |
| 加速度計 | **比力** f = (推力 + 空力)/m を機体座標で返す（旧: 重力の向きだけ） | EKF |

⚠ 加速度計を比力にするのが SIL にとって最大の変更。旧プラントは「機体がどう加速していても重力の向きだけ」を
返していたので、傾いて並進加速したときに EKF の姿勢が引っ張られる現象（実機の「偽の傾き」）を原理的に再現できなかった。

座標: 慣性 NED（z 下向き正）、機体 FRD（x 前・y 右・z 下）。モータ順は実機と同じ [FR, FL, MR, ML, RR, RL]。
パラメータの出典と確からしさは `Plant6Params` の各フィールドのコメントに書く（**同定済み / 推定 / 仮値** を区別）。
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

# --- 形状（nmpc_model.cpp kXm/kYm/kS と同じ） -------------------------------
KXM = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
KYM = np.array([0.050, -0.050, 0.10, -0.10, 0.050, -0.050])
KS = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, 1.0])      # ヨー反トルクの符号（機体が受ける向き）
SPIN = -KS                                            # ロータの回転向き（機体 z 軸まわり、+ は z 正回り）

# 飛行較正の推力曲線 Tf(d) = S(a d² + b d + c) [N/発]（20260920 の decision、ホバー duty 0.5506 @285 g）
TC_A, TC_B, TC_C, TC_S = -0.2265, 1.0525, 0.0452, 0.8389


def thrust_curve(d):
    d = np.clip(np.asarray(d, dtype=float), 0.0, 1.0)
    return np.where(d > 1e-6, TC_S * (TC_A * d * d + TC_B * d + TC_C), 0.0)


def thrust_inv(T):
    y = T / TC_S
    disc = max(TC_B * TC_B - 4 * TC_A * (TC_C - y), 0.0)
    return (-TC_B + math.sqrt(disc)) / (2 * TC_A)


@dataclass
class Plant6Params:
    # ---- 質量・慣性（nmpc_params.hpp と同じ。慣性は二線振り子、質量は実測） ----
    mass: float = 0.285
    gravity: float = 9.81
    Ix: float = 1.8e-3
    Iy: float = 1.9e-3
    Iz: float = 3.00e-3
    # 回転減衰（ファームと同形 C ω|ω|。⚠ 飛行ログ回帰では「識別不能」= 仮値）
    Cp: float = 0.025
    Cq: float = 0.025
    Cr: float = 1.0e-4
    # ---- 推力・反トルク ----
    Ct: float = 5.21e-7                 # [N/(rad/s)²] 静推力ベンチ（thrust_sysid_f1404.md）
    C_QT: float = 1.54e-8 / 5.21e-7     # CQ/Ct [m]（nmpc_params.hpp と同じ）
    motor_gain: tuple = (1.0,) * 6      # 発ごとの推力倍率（個体差・片側不足の模擬。既定 1）
    battery_sag_per_s: float = 0.0      # 推力の時間低下率 [1/s]（電池電圧の降下。既定 0）
    # ---- モータ ----
    motor_model: str = "rotor"          # "rotor": 回転数の一次遅れ / "duty": duty の一次遅れ（旧プラント互換）
    tau_up: float = 0.05                # [s] 加速側の時定数（実機同定 40–50 ms）
    tau_down: float = 0.05              # [s] 減速側（アクティブブレーキの無い ESC では大きくなる。⚠ 未同定）
    J_rotor: float = 0.0                # [kg m²] ロータ（プロペラ＋ベル）慣性。⚠ 推定 1.1e-6（3.6in 1.4 g プロペラ＋1404 ベル）
    # ---- 空力 ----
    rho: float = 1.225
    C_drag: tuple = (0.0, 0.0, 0.0)     # 形状抗力 C_u, C_v, C_w [N/(m/s)²]。⚠ 推定 (0.006, 0.006, 0.025) = ½ρ Cd A
    k_rotor_drag: float = 0.0           # [s/m] F = −k T v_xy。⚠ 推定 0.03（小型機の文献値 d≈0.3 1/s 相当）
    k_bf_roll: float = 0.0              # [N m/(m/s)] ブローバック。⚠ 推定 7.5e-3（0911 v6/v9 の回帰の換算、上限寄り）
    k_bf_pitch: float = 0.0             # 同 ⚠ 推定 2.8e-3
    prop_radius: float = 0.0457         # [m] GEMFAN 3630（直径 0.0914 m）
    ge_enable: bool = False             # 地面効果
    rotor_height: float = 0.04          # [m] 接地時の回転面の高さ（脚の高さ）。⚠ 推定
    ge_max: float = 1.3                 # 地面効果の上限倍率
    v_pitch_speed: float = 0.0          # [m/s] 軸流で推力 0 になる流速。⚠ 推定 11.5（ピッチ 3 in × 150 rev/s）。0 で無効
    # ---- 風 ----
    wind_ned: tuple = (0.0, 0.0, 0.0)   # [m/s] 一定風（NED、風が吹いていく向き）
    gust_sigma: float = 0.0             # [m/s] 突風の標準偏差（各軸）
    gust_tau: float = 1.0               # [s] 突風の相関時間
    # ---- その他 ----
    proc_sigma: float = 0.0             # [rad/s/√s] 角速度のランダムウォーク（モデル化していない揺れ）。スカラー or (p, q, r) の軸別タプル
    specific_force_acc: bool = False    # True: 加速度計 = 比力 / False: 重力の向きだけ（旧プラント）
    translation_coupled: bool = True    # False: 並進を積分しない（姿勢だけ、旧プラント互換）
    n_substeps: int = 2                 # 400 Hz 1 tick あたりの RK4 分割数

    @property
    def hover_duty(self) -> float:
        return float(thrust_inv(self.mass * self.gravity / 6.0))


# よく使う組み合わせ
def preset(name: str, **kw) -> Plant6Params:
    """'legacy': 旧 AttitudePlant 相当（並進は積分するが姿勢・センサに戻さない）
    'rigid': 6 自由度剛体 + 比力加速度計（空力なし）
    'full' : 推定値で全部入り（風なし）"""
    if name == "legacy":
        p = Plant6Params(motor_model="duty", specific_force_acc=False)
    elif name == "rigid":
        p = Plant6Params(motor_model="duty", specific_force_acc=True)
    elif name == "full":
        p = Plant6Params(motor_model="rotor", specific_force_acc=True, J_rotor=1.1e-6,
                         C_drag=(0.006, 0.006, 0.025), k_rotor_drag=0.03,
                         k_bf_roll=7.5e-3, k_bf_pitch=2.8e-3,
                         ge_enable=True, v_pitch_speed=11.5)
    else:
        raise ValueError(name)
    return replace(p, **kw)


# ---- クォータニオン（[q0,q1,q2,q3]、機体→NED の回転） ----------------------
def quat_from_euler(phi, theta, psi):
    cr, sr = math.cos(phi / 2), math.sin(phi / 2)
    cp, sp = math.cos(theta / 2), math.sin(theta / 2)
    cy, sy = math.cos(psi / 2), math.sin(psi / 2)
    return np.array([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
                     cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy])


def euler_from_quat(q):
    q0, q1, q2, q3 = q
    phi = math.atan2(2 * (q0 * q1 + q2 * q3), 1 - 2 * (q1 * q1 + q2 * q2))
    s = max(-1.0, min(1.0, 2 * (q0 * q2 - q3 * q1)))
    theta = math.asin(s)
    psi = math.atan2(2 * (q0 * q3 + q1 * q2), 1 - 2 * (q2 * q2 + q3 * q3))
    return phi, theta, psi


def rot_body_to_ned(q):
    """スライドの「速度の慣性空間への座標変換（クォータニオン表現）」そのもの。
    ⚠ スライドの Ẋ_e の第 1 項は q_4² になっているが q_3² の誤記。"""
    q0, q1, q2, q3 = q
    return np.array([
        [q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, 2 * (q1 * q2 - q0 * q3), 2 * (q1 * q3 + q0 * q2)],
        [2 * (q1 * q2 + q0 * q3), q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, 2 * (q2 * q3 - q0 * q1)],
        [2 * (q1 * q3 - q0 * q2), 2 * (q2 * q3 + q0 * q1), q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3],
    ])


def rot_body_to_ned_euler(phi, theta, psi):
    """スライドの「オイラー角表現」。テストでクォータニオン表現との一致を確認する。"""
    cf, sf, ct, st, cp, sp = math.cos(phi), math.sin(phi), math.cos(theta), math.sin(theta), math.cos(psi), math.sin(psi)
    return np.array([
        [ct * cp, sf * st * cp - cf * sp, cf * st * cp + sf * sp],
        [ct * sp, sf * st * sp + cf * cp, cf * st * sp - sf * cp],
        [-st, sf * ct, cf * ct],
    ])


# 状態ベクトル: pos(3) | vel_body(3) | quat(4) | rates(3) | motor(6)
IP, IV, IQ, IW, IM = slice(0, 3), slice(3, 6), slice(6, 10), slice(10, 13), slice(13, 19)
NXS = 19


class Plant6DOF:
    """SIL 用 6 自由度プラント。`AttitudePlant` と同じ参照用プロパティ（phi, theta, psi, rates, altitude, …）を持つ。"""

    def __init__(self, params: Plant6Params | None = None, dt: float = 0.0025, seed: int = 0):
        self.p = params or Plant6Params()
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.reset()

    # ---------------------------------------------------------------- 初期化
    def reset(self, phi=0.0, theta=0.0, psi=0.0):
        self.s = np.zeros(NXS)
        self.s[IQ] = quat_from_euler(phi, theta, psi)
        self.on_ground = True
        self.t = 0.0
        self.gust = np.zeros(3)
        self.duty_cmd = np.zeros(6)
        self._f_body = np.array([0.0, 0.0, -self.p.gravity])   # 比力（接地中は支持力）
        self._aux = {}

    def set_airborne(self, alt, duty, vel_body=(0.0, 0.0, 0.0)):
        """試験用: 高度 alt で、モータを duty の定常状態にして空中から始める。"""
        self.s[2] = -alt
        self.s[IV] = np.asarray(vel_body, dtype=float)
        self.s[IM] = self._motor_target(np.asarray(duty, dtype=float) * np.ones(6))
        self.on_ground = False

    # ---------------------------------------------------------------- モータ
    def _motor_target(self, duty):
        p = self.p
        if p.motor_model == "duty":
            return np.clip(duty, 0.0, 1.0)
        return np.sqrt(thrust_curve(duty) / p.Ct)                 # 定常回転数 [rad/s]

    def _thrusts(self, m):
        p = self.p
        if p.motor_model == "duty":
            T = thrust_curve(m)
        else:
            T = p.Ct * m * m
        T = T * np.asarray(p.motor_gain)
        if p.battery_sag_per_s > 0.0:
            T = T * max(0.5, 1.0 - p.battery_sag_per_s * self.t)
        return T

    # ---------------------------------------------------------------- 微分方程式
    def _deriv(self, s, target, wind_ned):
        p = self.p
        q = s[IQ]
        R = rot_body_to_ned(q)
        vb = s[IV]
        pw, qw, rw = s[IW]
        m = s[IM]

        # モータ（上げ・下げで時定数を分ける）
        tau = np.where(target >= m, p.tau_up, p.tau_down)
        mdot = (target - m) / tau
        T = self._thrusts(m)

        # 対気速度（機体座標）
        va = vb - R.T @ wind_ned
        # 推力の補正（地面効果・軸流）
        Tsum = float(T.sum())
        k_thr = 1.0
        alt = -s[2]
        if p.ge_enable:
            z = max(alt + p.rotor_height, 1e-3)
            k_thr *= min(p.ge_max, 1.0 / max(1.0 - (p.prop_radius / (4.0 * z)) ** 2, 1e-3))
        if p.v_pitch_speed > 0.0:
            v_in = -va[2]                                   # 上昇（w<0）でロータを下から抜ける流れ
            k_thr *= float(np.clip(1.0 - v_in / p.v_pitch_speed, 0.5, 1.5))
        T = T * k_thr
        Tsum *= k_thr

        # ---- 力（機体座標、重力以外） ----
        F = np.array([0.0, 0.0, -Tsum])
        Cu, Cv, Cw = p.C_drag
        F -= np.array([Cu, Cv, Cw]) * va * np.abs(va)
        if p.k_rotor_drag > 0.0:
            F[0] -= p.k_rotor_drag * Tsum * va[0]
            F[1] -= p.k_rotor_drag * Tsum * va[1]
        g_b = R.T @ np.array([0.0, 0.0, p.gravity])
        f_spec = F / p.mass                                  # 比力（加速度計が測る量）

        # ---- 並進（スライドの式: m(u̇+qw−rv) = … ） ----
        if p.translation_coupled:
            vdot = np.array([rw * vb[1] - qw * vb[2],
                             pw * vb[2] - rw * vb[0],
                             qw * vb[0] - pw * vb[1]]) + f_spec + g_b
            posdot = R @ vb
        else:
            vdot = np.zeros(3); posdot = np.zeros(3)

        # ---- モーメント ----
        L = float(np.sum(-KYM * T))
        M = float(np.sum(KXM * T))
        N = float(np.sum(KS * p.C_QT * T))
        if p.motor_model == "rotor" and p.J_rotor > 0.0:
            N += float(np.sum(KS * p.J_rotor * mdot))        # ロータ加減速の反トルク
            h = p.J_rotor * float(np.sum(SPIN * m))           # ロータ角運動量（機体 z）
            L += -qw * h                                       # −ω×h
            M += pw * h
        L += -p.k_bf_roll * va[1]                            # ブローバック: 右へ流れると左へ戻す
        M += p.k_bf_pitch * va[0]                            #              前へ流れると機首上げ

        wdot = np.array([
            (L - (p.Iz - p.Iy) * qw * rw - p.Cp * pw * abs(pw)) / p.Ix,
            (M - (p.Ix - p.Iz) * rw * pw - p.Cq * qw * abs(qw)) / p.Iy,
            (N - (p.Iy - p.Ix) * pw * qw - p.Cr * rw * abs(rw)) / p.Iz,
        ])
        q0, q1, q2, q3 = q
        qdot = 0.5 * np.array([-q1 * pw - q2 * qw - q3 * rw,
                               q0 * pw + q2 * rw - q3 * qw,
                               q0 * qw - q1 * rw + q3 * pw,
                               q0 * rw + q1 * qw - q2 * pw])
        ds = np.empty(NXS)
        ds[IP] = posdot; ds[IV] = vdot; ds[IQ] = qdot; ds[IW] = wdot; ds[IM] = mdot
        return ds, f_spec, Tsum, k_thr

    # ---------------------------------------------------------------- 1 tick
    def step(self, duty_cmd):
        p = self.p
        self.duty_cmd = np.clip(np.asarray(duty_cmd, dtype=float), 0.0, 1.0)
        target = self._motor_target(self.duty_cmd)
        # 突風（一次ガウス・マルコフ）
        if p.gust_sigma > 0.0:
            a = self.dt / p.gust_tau
            self.gust += -a * self.gust + p.gust_sigma * math.sqrt(2 * a) * self.rng.normal(size=3)
        wind = np.asarray(p.wind_ned, dtype=float) + self.gust

        n = max(1, int(p.n_substeps)); h = self.dt / n
        s = self.s
        for _ in range(n):
            k1, *_ = self._deriv(s, target, wind)
            k2, *_ = self._deriv(s + 0.5 * h * k1, target, wind)
            k3, *_ = self._deriv(s + 0.5 * h * k2, target, wind)
            k4, *_ = self._deriv(s + h * k3, target, wind)
            s = s + (h / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)
            s[IQ] /= np.linalg.norm(s[IQ])
        self.s = s
        self.t += self.dt

        _, f_spec, Tsum, k_thr = self._deriv(self.s, target, wind)
        self._aux = dict(T=Tsum, k_thr=k_thr, wind=wind)
        self._ground_contact(Tsum, f_spec)

        sig = np.broadcast_to(np.asarray(p.proc_sigma, dtype=float), (3,))   # スカラー or (p, q, r) 軸別
        if np.any(sig > 0.0) and not self.on_ground:
            self.s[IW] += sig * math.sqrt(self.dt) * self.rng.normal(size=3)
        return self.s

    def _ground_contact(self, Tsum, f_spec):
        """脚で立っている間は水平・静止に保ち、推力が重量を超えたら離床する（旧プラントと同じ扱い）。
        飛行中に地面へ下向きに着いたら止める（墜落判定は呼び出し側で行う）。"""
        p = self.p
        pos = self.s[IP]
        if self.on_ground:
            if Tsum > p.mass * p.gravity:
                self.on_ground = False                     # 離床
            else:
                _, _, psi = euler_from_quat(self.s[IQ])
                self.s[IQ] = quat_from_euler(0.0, 0.0, psi)
                self.s[IW] = 0.0
                self.s[IV] = 0.0
                pos[2] = 0.0
                self._f_body = np.array([0.0, 0.0, -p.gravity])   # 支持力（水平）
                return
        elif pos[2] >= 0.0 and self.vel[2] >= 0.0:
            pos[2] = 0.0                                   # 接地
            self.s[IV] = 0.0
            self.on_ground = True
            self._f_body = -rot_body_to_ned(self.s[IQ]).T @ np.array([0.0, 0.0, p.gravity])
            return
        self._f_body = f_spec

    # ---------------------------------------------------------------- 参照用（AttitudePlant 互換）
    @property
    def euler(self): return euler_from_quat(self.s[IQ])
    @property
    def phi(self): return self.euler[0]
    @property
    def theta(self): return self.euler[1]
    @property
    def psi(self): return self.euler[2]
    @property
    def rates(self): return self.s[IW].copy()
    @property
    def pos(self): return self.s[IP]
    @property
    def vel(self): return rot_body_to_ned(self.s[IQ]) @ self.s[IV]      # NED 速度
    @property
    def vel_body(self): return self.s[IV].copy()
    @property
    def altitude(self): return float(-self.s[2])
    @property
    def climb_rate(self): return float(-self.vel[2])
    @property
    def ground_speed(self): return float(np.hypot(*self.vel[:2]))
    @property
    def duty_actual(self):
        m = self.s[IM]
        if self.p.motor_model == "duty":
            return m.copy()
        return np.array([thrust_inv(t) if t > 0 else 0.0 for t in self.p.Ct * m * m])

    def specific_force_body(self):
        """加速度計が測る比力（機体 FRD）[m/s²]。specific_force_acc=False なら重力の向きだけを返す（旧プラント）。"""
        if not self.p.specific_force_acc:
            return -rot_body_to_ned(self.s[IQ]).T @ np.array([0.0, 0.0, self.p.gravity])
        return self._f_body.copy()

    def range_mm(self):
        """下向き測距（機体に固定）: 傾くと斜距離になる。"""
        phi, theta, _ = self.euler
        c = max(math.cos(phi) * math.cos(theta), 0.3)
        return float(np.clip(self.altitude / c * 1000.0, 0, 4000))

    @property
    def aux(self): return self._aux
