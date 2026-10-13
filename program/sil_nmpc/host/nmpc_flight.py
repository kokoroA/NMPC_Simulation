"""Benzen_NMPC を SIL で飛ばすための道具一式（2026-10-06）。

- 実機ファームは無改造（control.cpp / nmpc.cpp / ekf.cpp をそのままホストでビルドしたもの）
- プラントは実機の値: 285 g、飛行較正の推力曲線（ホバー duty 0.5506）、モータ一次遅れ 50 ms、
  ヨー減衰 Cr=1e-4、C_QT=CQ/Ct、慣性は nmpc_params.hpp と同じ（真値が不明なので倍率で振れる）
- 実機に近づけるオプション: ジャイロ・加速度計の雑音、機体外乱（角速度のランダムウォーク）、出力遅延
- 操縦者: スロットルは高度 PD を模擬（実機に高度制御は無いので「上手な操縦者」の代わり）。
  スティックは姿勢参照のステップ。
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass

import numpy as np

import rc
from plant import AttitudePlant, PlantParams
from plant6dof import Plant6DOF, Plant6Params
from sensors import GRAV, dcm_body_from_ned, quat_from_euler as _q_from_e
from sim import ClosedLoop, DT

# ---- 実機 Benzen_NMPC の定数（control.cpp / radio.hpp の写し）----------------
T_REF_GAIN = 0.5          # control.cpp: T_ref = 0.5*BATTERY*(ch3-min)/(max-min)
BATTERY = 7.4
T_REF_COE = 0.7           # control.cpp:131
DUTY_PER_RATIO = T_REF_GAIN * BATTERY * T_REF_COE      # = 2.59
CH_FTC, CH_LOG, CH_SERVO = 5, 7, 4                     # radio.hpp: FTC 5, LOG 7, SERVO 4
SW_LOW, SW_HIGH = 144, 1680
LOG_COLS = ["phi", "theta", "p", "q", "r", "phi_ref", "theta_ref", "psi_ref", "T_ref", "u_base",
            "u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl", "iter_us", "loop_us", "cycles", "failsafe", "ftc",
            "acc_norm", "ekf_bp", "ekf_bq", "ekf_br", "ekf_dphi", "ekf_dtheta", "lam_norm", "ulin_norm",
            "solver_code", "tr1_c", "secant_nsq", "tr1_skip_run"]
FS_NAMES = {0: "なし", 2: "受信途絶", 3: "姿勢超過", 4: "角速度超過", 5: "過大加速度", 6: "受信機FS", 7: "フレーム欠落"}


# 修正前の SIL が motor i として読んでいた PWM 出力は、Benzen_NMPC では motor LEGACY_PERM[i] のもの
#   FR←front A(=RR), FL←middle A(=ML), MR←rear A(=RL), ML←rear B(=FL), RR←front B(=FR), RL←middle B(=MR)
LEGACY_PERM = [4, 3, 5, 1, 0, 2]


def rc_base(log_on=True, ftc_on=False):
    """スティック中立・スロットル最低。⚠ 元の rc.neutral() は ch5(FTC) を中央 1003 にしてしまい FTC が入る。"""
    ch = rc.neutral()
    ch[CH_SERVO] = SW_LOW
    ch[CH_FTC] = SW_HIGH if ftc_on else SW_LOW       # >500 で FTC、<200 で解除
    ch[6] = SW_LOW
    ch[CH_LOG] = SW_HIGH if log_on else SW_LOW       # LOG > SW_MID で 100Hz ログ記録
    return ch


# ---- 実機の推力曲線（飛行較正）------------------------------------------------
TC_A, TC_B, TC_C, TC_S = -0.2265, 1.0525, 0.0452, 0.8389


def thrust_curve(d):
    d = np.clip(np.asarray(d, dtype=float), 0.0, 1.0)
    return np.where(d > 1e-6, TC_S * (TC_A * d * d + TC_B * d + TC_C), 0.0)


def thrust_inv(T):
    y = T / TC_S
    disc = max(TC_B * TC_B - 4 * TC_A * (TC_C - y), 0.0)
    return (-TC_B + math.sqrt(disc)) / (2 * TC_A)


@dataclass
class RealPlantParams(PlantParams):
    mass: float = 0.285
    Cp: float = 0.025
    Cq: float = 0.025
    Cr: float = 1.0e-4
    C_QT: float = 1.54e-8 / 5.21e-7
    T_max: float = 0.75                 # 推力曲線を使うので thrust_of では使わない（ログ用）
    motor_tau: float = 0.05
    proc_sigma: float = 0.0             # 機体外乱 [rad/s/√s]（角速度へのランダムウォーク）。実機の揺れ相当は 0.5
    inertia_scale: tuple = (1.0, 1.0, 1.0)

    def thrust_of(self, duty):
        return thrust_curve(duty)

    @property
    def hover_duty(self) -> float:
        return float(thrust_inv(self.mass * self.gravity / 6.0))


class RealPlant(AttitudePlant):
    """接地中は機体を水平に保持し（脚で立っている）、推力が重量を超えたら離床する。"""

    def __init__(self, params: RealPlantParams, dt=DT, seed=0):
        sx, sy, sz = params.inertia_scale
        params.Ix *= sx; params.Iy *= sy; params.Iz *= sz
        super().__init__(params, dt)
        self.rng = np.random.default_rng(seed)

    def step(self, duty_cmd):
        p = self.p
        if self.on_ground and float(np.sum(p.thrust_of(self.duty_actual))) < p.mass * p.gravity:
            self.x[0] = 0.0; self.x[1] = 0.0; self.x[3:6] = 0.0
        out = super().step(duty_cmd)
        sig = np.broadcast_to(np.asarray(p.proc_sigma, dtype=float), (3,))   # スカラー or (p, q, r) 軸別
        if np.any(sig > 0.0) and not self.on_ground:
            self.x[3:6] += sig * math.sqrt(self.dt) * self.rng.normal(size=3)
        return out


class NmpcLoop(ClosedLoop):
    def __init__(self, plant_params, seed=0, gyro_sigma=0.0, acc_sigma_mg=0.0,
                 out_delay_ticks=0, rebuild_hold_ticks=0, legacy_order=False, gyro_bias_rw=0.0, log=True):
        """out_delay_ticks: core1 の 1 周期（実機 loop_us≈10 ms）ぶん duty が遅れて出ることの模擬。
        rebuild_hold_ticks: フル再構築の周期は実機で 31–45 ms かかり、その間 duty は更新されない。
        SIL の core1 は仮想時間ゼロで終わるので、プラント側で「出力を据え置く」ことで近似する。
        （⚠ 実機では据え置き中の 100Hz 周期そのものが無いが、SIL では core1 が回り続けて内部状態は進む）"""
        super().__init__(None, log=log)
        self.rebuild_hold_ticks = rebuild_hold_ticks
        # legacy_order=True: 修正前の SIL（Benzen_new 用 slice 表）がプラントに渡していた順を再現する（図・回帰確認用）
        self.legacy_order = legacy_order
        self._hold_left = 0
        self._last_applied = None
        self._rel_seen = None
        self.n_holds = 0
        self.order_checks = 0      # モータ順検査の回数と、PWM 出力が firmware の u_* と食い違った回数
        self.order_bad = 0
        # 2026-10-08: Plant6Params を渡すと 6 自由度プラント（並進連成・比力加速度計・空力）を使う
        if isinstance(plant_params, Plant6Params):
            self.plant = Plant6DOF(plant_params, dt=DT, seed=seed)
        else:
            self.plant = RealPlant(plant_params, dt=DT, seed=seed)
        self.is6 = isinstance(self.plant, Plant6DOF)
        self.rng = np.random.default_rng(seed + 1000)
        self.gyro_sigma = gyro_sigma
        # ジャイロバイアスのランダムウォーク [rad/s/√s]（電源投入からずっと漂う。EKF の加速度計の重みを下げる案の代償を見るため）
        self.gyro_bias_rw = gyro_bias_rw
        self.gyro_bias = np.zeros(3)
        self.acc_sigma = acc_sigma_mg
        self.delay = deque(maxlen=max(out_delay_ticks, 0) + 1)
        self.out_delay_ticks = out_delay_ticks
        self._log_rel0 = None            # LOG を入れた時点の core1 起動回数（ログ行数の基準）
        self.set_rc(rc_base(log_on=False))

    def set_rc(self, ch):
        # LOG の立ち上がりで firmware は LogRow=0 から書き直す。行数は以後の core1 起動回数で数える
        # （LogRow は control.cpp の static で外から読めない。loop_us は仮想時間なので常に 0 で目印にならない）。
        log_on = ch[CH_LOG] > (SW_LOW + SW_HIGH) // 2
        was_on = getattr(self, "ch", None) is not None and self.ch[CH_LOG] > (SW_LOW + SW_HIGH) // 2
        super().set_rc(ch)
        if log_on and not was_on:
            self._log_rel0 = self.sil.sched_counts()[0]
        elif not log_on:
            self._log_rel0 = None

    def step(self):
        pl = self.plant
        p, q, r = pl.rates
        extra = None
        if self.is6 and pl.p.specific_force_acc:
            # 加速度計 = 比力。sensors は「a_body = 重力(機体) + extra」を −比力 として生値にするので、その差を渡す
            g_b = dcm_body_from_ned(_q_from_e(pl.phi, pl.theta, pl.psi)) @ np.array([0.0, 0.0, GRAV])
            extra = -pl.specific_force_body() - g_b
        acc, gyr, mag = self.sensors.raw_from_state(pl.phi, pl.theta, pl.psi, p, q, r, accel_body_extra=extra)
        if self.gyro_bias_rw > 0:
            self.gyro_bias += self.gyro_bias_rw * math.sqrt(DT) * self.rng.normal(size=3)
            gyr = [g + b * 180 / math.pi * 1000 for g, b in zip(gyr, self.gyro_bias)]
        if self.gyro_sigma > 0:
            gyr = [g + self.rng.normal(0.0, self.gyro_sigma * 180 / math.pi * 1000) for g in gyr]
        if self.acc_sigma > 0:
            acc = [a + self.rng.normal(0.0, self.acc_sigma) for a in acc]
        self.sil.set_imu(acc, gyr, mag)
        self.sil.set_range_mm(int(pl.range_mm()) if self.is6 else int(np.clip(pl.altitude * 1000.0, 0, 4000)))
        self.sil.tick()
        duty = self.sil.duty()
        if self.legacy_order:
            duty = [duty[i] for i in LEGACY_PERM]
        self.delay.append(duty)
        applied = self.delay[0] if self.out_delay_ticks > 0 else duty
        # LOG 中は core1 が書いた最新行を読む:
        #   (a) 再構築周期（solver_code の十の位 = 1）の検出
        #   (b) モータ順の検査: core1 が起動しなかった tick の PWM 出力は「最新行の u_*」に一致するはず
        #       （⚠ 2026-10-08 に SIL の slice→モータ表が Benzen_new 用のままで 6 発が入れ替わっていた）
        if self._log_rel0 is not None:
            rel = self.sil.sched_counts()[0]
            if rel != self._rel_seen:
                self._rel_seen = rel
                n = rel - self._log_rel0
                if n > 0:
                    row = self.sil.logrow((n - 1) % (65280 // 33))
                    self._last_u = np.clip(np.array(row[10:16]), 0.01, 0.97)
                    if self.rebuild_hold_ticks > 0 and int(row[29]) // 10 == 1 and self._hold_left == 0:
                        self._hold_left = self.rebuild_hold_ticks
                        self.n_holds += 1
            elif getattr(self, "_last_u", None) is not None and np.ptp(self._last_u) > 0.01:
                # 6 発が揃っている周期は順序が入れ替わっても区別できないので数えない
                err = float(np.max(np.abs(np.asarray(duty) - self._last_u)))
                self.order_checks += 1
                if err > 0.003:
                    self.order_bad += 1
        if self._hold_left > 0 and self._last_applied is not None:
            applied = self._last_applied
            self._hold_left -= 1
        self._last_applied = applied
        pl.step(applied)
        self.t += DT
        if self.log_enabled:
            st = self.sil.status()
            self.rows.append(dict(t=self.t, phi=pl.phi, theta=pl.theta, psi=pl.psi, p=p, q=q, r=r,
                                  alt=pl.altitude, vz=-pl.vel[2], arm=st['arm'], gate=st['gate'], fs=st['fs'],
                                  fs_cause=st['fs_cause'], thr=self.ch[rc.THROTTLE],
                                  **{f"d{i}": d for i, d in enumerate(applied)},
                                  **(self._log6(pl) if self.is6 else {})))
        return duty

    @staticmethod
    def _log6(pl):
        """6 自由度プラントだけが持つ量（位置・速度・比力・風）"""
        vn = pl.vel; vb = pl.vel_body; f = pl.specific_force_body(); w = pl.aux.get('wind', np.zeros(3))
        return dict(pn=pl.pos[0], pe=pl.pos[1], vn=vn[0], ve=vn[1], vd=vn[2], ub=vb[0], vb=vb[1], wb=vb[2],
                    fx=f[0], fy=f[1], fz=f[2], wind_n=w[0], wind_e=w[1], T=pl.aux.get('T', 0.0))

    def arm_nmpc(self, max_gate_s=8.0):
        """実機と同じ手順: ウォームアップ → ロック解除ジェスチャ → スティック中立 → 離陸ゲートが開くまで待つ。"""
        self.run(6010)
        st = self.sil.status()
        if st['arm'] != 3:
            raise RuntimeError(f"ウォームアップ後 Arm_flag={st['arm']}（期待 3）")
        g = rc.arm_gesture()
        for i in (CH_SERVO, CH_FTC, 6):
            g[i] = SW_LOW
        g[CH_LOG] = SW_LOW
        self.set_rc(g); self.run(80)
        self.set_rc(rc_base(log_on=False)); self.run(80)
        st = self.sil.status()
        if st['arm'] != 2:
            raise RuntimeError(f"アーム失敗 {st}")
        n = 0
        while not self.sil.status()['gate'] and n < int(max_gate_s / DT):
            self.step(); n += 1
        if not self.sil.status()['gate']:
            raise RuntimeError(f"離陸ゲートが開かない {self.sil.status()}")
        return self.t

    def logdata(self):
        """LOG を入れてから書かれた行を古い順に返す（リングが一周していれば並べ替える）。"""
        L = self.sil.logdata()
        if self._log_rel0 is None:
            return L[:0]
        n = self.sil.sched_counts()[0] - self._log_rel0
        rows = L.shape[0]
        if n <= rows:
            return L[:n]
        head = n % rows
        return np.vstack([L[head:], L[:head]])


STICK_FULL_RAD = 0.3 * math.pi   # control.cpp update_attitude_refs_from_sticks（⚠ rc.py の 0.8π は旧値）


def with_sticks(ch, phi_ref=0.0, theta_ref=0.0, yaw_rate_ref=0.0):
    """Phi_ref / Theta_ref / Psi_ref(ヨー率) [rad, rad/s] になるスティック値。中心はアーム時ラッチ値＝中立。"""
    out = list(ch)
    for idx, val, lo, hi in ((rc.AILERON, phi_ref, 368, 1638), (rc.ELEVATOR, theta_ref, 368, 1680),
                             (rc.RUDDER, yaw_rate_ref, 368, 1680)):
        mid = (lo + hi) // 2
        out[idx] = int(round(mid + val / STICK_FULL_RAD * (hi - lo) / 2.0))
    return out


def throttle_ratio_for_duty(u):
    return u / DUTY_PER_RATIO


class Pilot:
    """スロットルを高度 PD で動かす操縦者モデル（実機に高度制御は無い）。スティック速度は制限する。"""

    def __init__(self, mass, kp=0.03, kd=0.04, rate=0.6):
        self.r_hover = throttle_ratio_for_duty(thrust_inv(mass * 9.81 / 6.0))
        self.kp, self.kd, self.rate = kp, kd, rate
        self.r = 0.0

    def update(self, h_ref, h, vz, dt):
        tgt = self.r_hover + self.kp * (h_ref - h) - self.kd * vz
        tgt = float(np.clip(tgt, 0.0, 0.40))
        dr = float(np.clip(tgt - self.r, -self.rate * dt, self.rate * dt))
        self.r += dr
        return self.r
