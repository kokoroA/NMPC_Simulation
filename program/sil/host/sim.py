"""SIL 閉ループ: プラント <-> 実機ファーム。

    プラント状態 -> センサ生値 -> ファーム(400Hz) -> duty -> プラント積分

ファーム側から見ると 1 tick 前の状態を観測することになるが、
これは実機でも同じ（センサ読取と出力の間に 1 周期分の遅れがある）。
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

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
                "phi_true": pl.phi, "theta_true": pl.theta, "psi_true": pl.psi,
                "p_true": p, "q_true": q, "r_true": r,
                "phi_est": st["phi"], "theta_est": st["theta"],
                "phi_ref": st["phi_ref"], "theta_ref": st["theta_ref"],
                "Pref": st["Pref"], "Qref": st["Qref"], "Rref": st["Rref"],
                "T_ref": st["T_ref"], "alt": pl.altitude,
                "arm": self.sil.arm_flag,
                **{f"duty_{n.lower()}": d for n, d in zip(("FR", "FL", "MR", "ML", "RR", "RL"), duty)},
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

    def disturb_rates(self, dp=0.0, dq=0.0, dr=0.0):
        """突風相当の角速度インパルスを与える。

        ⚠ 姿勢そのものを書き換えてはいけない。角速度を伴わない姿勢変化は
           EKF に「ジャイロバイアス」と解釈され、周期 20 秒級の緩い振動を生む。
           これは制御の問題ではなくテストの物理が間違っているだけ。
        """
        self.plant.x[3] += dp
        self.plant.x[4] += dq
        self.plant.x[5] += dr

    def to_dataframe(self):
        try:
            import pandas as pd
        except ImportError:
            return None
        return pd.DataFrame(self.rows)
