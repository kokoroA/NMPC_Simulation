"""ヘキサコプタの姿勢ダイナミクス（プラント）。

**出典は実機ファーム自身**（Benzen_NMPC/nmpc_model.cpp, nmpc_params.hpp）。
シミュレータ側で勝手にモデルを作らず、実機が前提としている式をそのまま使う。
そうしないと「SIL では飛ぶが実機では飛ばない」の原因がモデル差なのか
制御なのか切り分けられなくなる。

モータ順は実機と同じ [FR, FL, MR, ML, RR, RL]。

姿勢のみを状態に持つ（実機の制御対象と一致）。位置・速度は可視化用に
開ループで積分するだけで、制御には一切フィードバックしない。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

# --- 実機モータ配置 [m]（nmpc_model.cpp kXm/kYm/kS） -----------------------
# x: 前方正, y: 右正
KXM = np.array([0.085, 0.085, 0.0, 0.0, -0.085, -0.085])
KYM = np.array([0.050, -0.050, 0.10, -0.10, 0.050, -0.050])
KS = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, 1.0])   # 反トルクの符号
MOTOR_NAMES = ("FR", "FL", "MR", "ML", "RR", "RL")


@dataclass
class PlantParams:
    """nmpc_params.hpp の physical:: と対応させること。乖離させたら理由を記録する。"""
    Ix: float = 1.8e-3          # [kg m^2]
    Iy: float = 1.9e-3
    Iz: float = 3.00e-3
    Cp: float = 0.01            # 空力ダンピング（角速度^2 比例）
    Cq: float = 0.025
    Cr: float = 0.005
    mass: float = 0.356         # [kg]
    gravity: float = 9.81
    C_QT: float = 0.03614       # CQ/Ct [m]

    # ⚠ T_max は **未検証の較正パラメータ**。
    #    nmpc_params.hpp に「暫定値：実機ホバー測定後に更新推奨」とある値をそのまま置いている。
    #    実測ではもっと大きい可能性が高い。実機のホバー duty を測ったら必ず更新すること。
    #    ここが違うと自動機動(M5)の軌道がまるごとずれる。
    T_max: float = 2.04         # duty=1 での 1 モータ推力 [N]

    # 推力カーブの指数。1.0 = nmpc_model.cpp と同じ線形仮定（既定）。
    # 物理的にはプロペラ推力は回転数の 2 乗に比例し、duty->回転数がほぼ線形なので
    # 実機は 2.0 に近いはず。実機データが取れるまで**ファームの仮定に合わせて 1.0** にしておく。
    # 変える場合は必ず decisions に理由を残すこと。
    thrust_exponent: float = 1.0

    motor_tau: float = 0.0      # モータ一次遅れ [s]（0 で無効。M5 で同定値を入れる）

    # ---- 並進（可視化用。制御にはフィードバックしない） -------------------
    # ⚠ NMPC_simulation の PlantModel にも**並進ドラッグは無い**
    #    （F_world は時間スケジュール型の外乱専用）。したがってここで足す。
    #    姿勢には一切影響しないので、既存の姿勢検証結果は変わらない。
    #
    # 二次抗力: F = -drag_k * |v| * v  [N]
    # drag_k = 0.010 は「15deg 傾けて水平推力 ~0.9N のとき終端速度 ~9.5 m/s」
    # になる値。**実機で同定した値ではない**。軌跡の形を見るための概算。
    drag_k: float = 0.010       # [N/(m/s)^2] 0 で無効（従来の発散する挙動）

    # ---- NMPC_simulation の PlantModel から移植（既定 OFF） ---------------
    # 有効にすると姿勢にも影響するため、既存の検証結果と比較したいときは OFF のまま。
    w_induced_hover: float = 0.0   # [m/s] PKG-E3 上昇中の推力低下。12.0 で有効
    aero_factor_min: float = 0.30
    aero_factor_max: float = 1.20
    ge_height: float = 0.0         # [m] PKG-E7 地面効果。0.5 で有効
    ge_max_boost: float = 0.15

    def thrust_of(self, duty):
        """duty -> 1 モータ推力 [N]。"""
        return self.T_max * np.power(np.clip(duty, 0.0, 1.0), self.thrust_exponent)

    @property
    def hover_duty(self) -> float:
        """ホバーに必要な 1 モータ duty。"""
        need = self.mass * self.gravity / (6.0 * self.T_max)
        return float(need ** (1.0 / self.thrust_exponent))

    def T_max_for_hover_at(self, duty: float) -> float:
        """指定 duty でホバーするために必要な T_max（較正用の逆算）。"""
        return self.mass * self.gravity / (6.0 * duty ** self.thrust_exponent)


class AttitudePlant:
    """nmpc_model.cpp の model_f をそのまま実装したもの。

    状態: [phi, theta, psi, p, q, r]
      psi は実機 EKF では推定されないが、可視化とヨーレート積分の確認に使う。
    """

    def __init__(self, params: PlantParams | None = None, dt: float = 0.0025):
        self.p = params or PlantParams()
        self.dt = dt
        self.reset()

    def reset(self, phi=0.0, theta=0.0, psi=0.0):
        self.x = np.array([phi, theta, psi, 0.0, 0.0, 0.0], dtype=float)
        self.duty_actual = np.full(6, 0.0)     # モータ遅れ後の duty
        self.pos = np.zeros(3)                 # NED [m]（開ループ、可視化用）
        self.vel = np.zeros(3)
        self.on_ground = True

    # --- 動力学 -----------------------------------------------------------
    def _xdot(self, x, duty):
        p_ = self.p
        phi, theta, psi, wp, wq, wr = x

        thrust = p_.thrust_of(duty)                    # [N] 各モータ
        # nmpc_model.cpp: B_c(roll)= -kYm*T_max/Ix, B_c(pitch)= +kXm*T_max/Iy,
        #                 B_c(yaw) = +kS*C_QT*T_max/Iz
        L = float(np.sum(-KYM * thrust))               # ロールモーメント [N m]
        M = float(np.sum(+KXM * thrust))               # ピッチモーメント
        N = float(np.sum(+KS * p_.C_QT * thrust))      # ヨーモーメント

        sphi, cphi = np.sin(phi), np.cos(phi)
        ttheta = np.tan(theta)
        ctheta = np.cos(theta)

        phi_dot = wp + wq * sphi * ttheta + wr * cphi * ttheta
        theta_dot = wq * cphi - wr * sphi
        psi_dot = (wq * sphi + wr * cphi) / max(ctheta, 1e-6)

        p_dot = (L - (p_.Iz - p_.Iy) * wq * wr - p_.Cp * wp * abs(wp)) / p_.Ix
        q_dot = (M - (p_.Ix - p_.Iz) * wr * wp - p_.Cq * wq * abs(wq)) / p_.Iy
        r_dot = (N - (p_.Iy - p_.Ix) * wp * wq - p_.Cr * wr * abs(wr)) / p_.Iz

        return np.array([phi_dot, theta_dot, psi_dot, p_dot, q_dot, r_dot])

    def step(self, duty_cmd):
        """duty [FR,FL,MR,ML,RR,RL] を 1 ステップ積分する。RK4。"""
        duty_cmd = np.clip(np.asarray(duty_cmd, dtype=float), 0.0, 1.0)

        # モータ一次遅れ（tau=0 なら即応）
        if self.p.motor_tau > 0.0:
            a = self.dt / (self.p.motor_tau + self.dt)
            self.duty_actual += a * (duty_cmd - self.duty_actual)
        else:
            self.duty_actual = duty_cmd

        d, x = self.duty_actual, self.x
        k1 = self._xdot(x, d)
        k2 = self._xdot(x + 0.5 * self.dt * k1, d)
        k3 = self._xdot(x + 0.5 * self.dt * k2, d)
        k4 = self._xdot(x + self.dt * k3, d)
        self.x = x + (self.dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4)

        self._integrate_translation()
        return self.x

    def total_thrust(self):
        """全モータ推力の合計 [N]。PKG-E3 / E7 を有効にしていれば反映する。"""
        p_ = self.p
        T = float(np.sum(p_.thrust_of(self.duty_actual)))
        # PKG-E3: 上昇中はプロペラへの流入で推力が落ちる
        if p_.w_induced_hover > 1e-9:
            v_up = -self.vel[2]                     # 上向き正 [m/s]
            f = 1.0 - v_up / p_.w_induced_hover
            T *= float(np.clip(f, p_.aero_factor_min, p_.aero_factor_max))
        # PKG-E7: 地面効果
        if p_.ge_height > 1e-9:
            alt = max(-self.pos[2], 0.0)
            if alt < p_.ge_height:
                T *= 1.0 + p_.ge_max_boost * (1.0 - alt / p_.ge_height)
        return T

    def _integrate_translation(self):
        """位置・速度の積分（可視化専用。制御には戻さない）。

        NMPC_simulation の PlantModel と同じ並進式を使い、
        **そこには無い二次抗力**を足している。抗力が無いと速度が際限なく
        増え、軌跡が地上経路として意味を持たなくなる。
        """
        phi, theta, psi = self.x[0], self.x[1], self.x[2]
        T = self.total_thrust()
        cphi, sphi = np.cos(phi), np.sin(phi)
        cth, sth = np.cos(theta), np.sin(theta)
        cps, sps = np.cos(psi), np.sin(psi)

        # 機体 -Z 方向の推力を NED へ（PlantModel の ax0/ay0/a_z と同形）
        m = self.p.mass
        acc_ned = np.array([
            -(T / m) * (cps * sth * cphi + sps * sphi),
            -(T / m) * (sps * sth * cphi - cps * sphi),
            self.p.gravity - (T / m) * (cth * cphi),
        ])

        # 二次抗力（NMPC_simulation には無い。ここで追加）
        if self.p.drag_k > 0.0:
            v = self.vel
            speed = float(np.linalg.norm(v))
            if speed > 1e-6:
                acc_ned -= (self.p.drag_k * speed / m) * v

        self.vel += acc_ned * self.dt
        self.pos += self.vel * self.dt

        # 接地（Down 正なので pos[2] > 0 が地面より下）
        if self.pos[2] > 0.0:
            self.pos[2] = 0.0
            if self.vel[2] > 0.0:      # 下向き速度だけ止める（再離陸できるように）
                self.vel[2] = 0.0
            self.vel[0] = 0.0          # 接地摩擦で水平も止まる
            self.vel[1] = 0.0
            self.on_ground = True
        else:
            self.on_ground = False

    # --- 参照用 -----------------------------------------------------------
    @property
    def phi(self): return self.x[0]
    @property
    def theta(self): return self.x[1]
    @property
    def psi(self): return self.x[2]
    @property
    def rates(self): return self.x[3:6]
    @property
    def altitude(self): return -self.pos[2]
    @property
    def ground_speed(self): return float(np.hypot(self.vel[0], self.vel[1]))
    @property
    def climb_rate(self): return float(-self.vel[2])
