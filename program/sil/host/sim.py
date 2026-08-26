"""SIL 閉ループ: プラント <-> 実機ファーム。

    プラント状態 -> センサ生値 -> ファーム(400Hz) -> duty -> プラント積分

ファーム側から見ると 1 tick 前の状態を観測することになるが、
これは実機でも同じ（センサ読取と出力の間に 1 周期分の遅れがある）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

import logger as _logger
import rc
from plant import AttitudePlant, PlantParams
from sensors import SensorModel, euler_rates_to_body_rates
from sil_bridge import Sil

DT = 0.0025  # 400 Hz


class ClosedLoop:
    def __init__(self, plant_params: PlantParams | None = None, log: bool = True):
        self.plant = AttitudePlant(plant_params, dt=DT)
        self.sensors = SensorModel()
        self.sil = Sil()
        self.ch = rc.neutral()
        self.sil.set_rc(self.ch)
        self.t = 0.0
        self.log_enabled = log
        self.rows: list[dict] = []

    # --- 入力 -------------------------------------------------------------
    def set_rc(self, ch):
        self.ch = list(ch)
        self.sil.set_rc(self.ch)

    # --- 1 tick -----------------------------------------------------------
    def step(self):
        pl = self.plant
        # プラント状態 -> センサ生値（角速度は機体角速度そのもの）
        p, q, r = pl.rates
        acc, gyr, mag = self.sensors.raw_from_state(pl.phi, pl.theta, pl.psi, p, q, r)
        self.sil.set_imu(acc, gyr, mag)
        self.sil.set_range_mm(int(np.clip(pl.altitude * 1000.0, 0, 4000)))
        self.sil.tick()

        duty = self.sil.duty()
        pl.step(duty)
        self.t += DT

        if self.log_enabled:
            st = self.sil.state()
            self.rows.append({
                "t": self.t,
                # --- 真値（プラント）---
                "phi_true": pl.phi, "theta_true": pl.theta, "psi_true": pl.psi,
                "p_true": p, "q_true": q, "r_true": r,
                "pos_n": pl.pos[0], "pos_e": pl.pos[1], "alt": pl.altitude,
                "vel_n": pl.vel[0], "vel_e": pl.vel[1], "vel_d": pl.vel[2],
                # --- ファーム内部 ---
                "phi_est": st["phi"], "theta_est": st["theta"], "psi_est": st["psi"],
                "phi_ref": st["phi_ref"], "theta_ref": st["theta_ref"], "psi_ref": st["psi_ref"],
                "p_est": st["p"], "q_est": st["q"], "r_est": st["r"],
                "Pref": st["Pref"], "Qref": st["Qref"], "Rref": st["Rref"],
                "P_com": st["P_com"], "Q_com": st["Q_com"], "R_com": st["R_com"],
                "T_ref": st["T_ref"], "T_stick": st["T_stick"],
                "arm": self.sil.arm_flag, "mode": self.sil.flight_mode,
                # --- 操作入力 ---
                "ch_rudder": self.ch[0], "ch_elevator": self.ch[1],
                "ch_throttle": self.ch[2], "ch_aileron": self.ch[3],
                # --- 出力 ---
                **{f"duty_{n.lower()}": d for n, d in zip(("FR", "FL", "MR", "ML", "RR", "RL"), duty)},
                # --- 計算コスト（ホスト実測。実機時間ではない）---
                "host_core0_ns": self.sil.last_loop_ns,
                "host_core1_ns": self.sil.last_core1_ns,
            })
        return duty

    def run(self, n):
        for _ in range(n):
            self.step()

    def run_seconds(self, sec):
        self.run(int(round(sec / DT)))

    # --- 定型シーケンス ---------------------------------------------------
    def arm(self, verbose=False):
        """実機と同じ手順でアームまで進める。

        1) BiasCounter が KALMANWAIT(6000) を超えるまで待つ -> Arm_flag=3
        2) lock_com ジェスチャを 50 回超保持して解除 -> Arm_flag=2
        """
        self.run(6010)
        if self.sil.arm_flag != 3:
            raise RuntimeError(f"ウォームアップ後 Arm_flag={self.sil.arm_flag} (期待 3)")
        self.set_rc(rc.arm_gesture())
        self.run(80)
        self.set_rc(rc.neutral())
        self.run(10)
        if self.sil.arm_flag != 2:
            raise RuntimeError(f"アーム失敗 Arm_flag={self.sil.arm_flag} (期待 2)")
        if verbose:
            print(f"  アーム完了 t={self.t:.2f}s")

    @property
    def thrust_delta_ratio(self) -> float:
        """上昇旋回の推力補正比を T_ref から逆算する。

        control.cpp:995  T_ref = 0.5*BATTERY*ratio * (1 + g_thrust_delta_ratio)
        g_thrust_delta_ratio は static で外部から読めないため、こう求める。
        """
        import firmware_consts as fc
        lo, hi = rc.CH_MIN[rc.THROTTLE], rc.CH_MAX[rc.THROTTLE]
        ratio = (self.ch[rc.THROTTLE] - lo) / float(hi - lo)
        base = fc.T_REF_THROTTLE_GAIN * fc.BATTERY_VOLTAGE * ratio
        if base <= 1e-9:
            return 0.0
        return self.sil.state()["T_ref"] / base - 1.0

    def disturb_rates(self, dp=0.0, dq=0.0, dr=0.0):
        """突風相当の角速度インパルスを与える。

        ⚠ 姿勢そのものを書き換えてはいけない。角速度を伴わない姿勢変化は
           EKF に「ジャイロバイアス」と解釈され、周期 20 秒級の緩い振動を生む。
           これは制御の問題ではなくテストの物理が間違っているだけ。
        """
        self.plant.x[3] += dp
        self.plant.x[4] += dq
        self.plant.x[5] += dr

    # --- 保存 -------------------------------------------------------------
    def save_run(self, note=None, name=None, csv=False):
        """ログを runs/ 配下へ保存する。meta にファームの git hash 等も残す。"""
        lg = _logger.RunLogger()
        for r in self.rows:
            lg.append(r)
        lo, hi = rc.CH_MIN[rc.THROTTLE], rc.CH_MAX[rc.THROTTLE]
        thr_ratio = (self.ch[rc.THROTTLE] - lo) / float(hi - lo)
        meta = _logger.build_meta(
            self.plant.p,
            throttle=round(thr_ratio, 4),
            note=note,
            extra={"dt": DT, "sim_seconds": self.t,
                   "throttle_raw": self.ch[rc.THROTTLE]},
        )
        return lg.save(meta, name=name, csv=csv)
