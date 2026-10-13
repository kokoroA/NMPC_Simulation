"""Benzen_NMPC（無改造ファーム）の耐故障制御（FTC スイッチで 1 発停止）を SIL で 1 回飛ばして npz に保存する（2026-10-13）。

1 プロセス 1 インスタンス（SIL の制約）なので、条件ごとに別プロセスで呼ぶこと。

    SIL_BUILD_DIR=<build> python3 run_ftc_eval.py --mass 0.225 --motor FR --mode air --cond shake --seed 0 --out x.npz

実機の FTC 実験と同じく「ch5 の FTC スイッチ = モータ停止と NMPC への通知が同時」が既定。
--detect-delay を正にすると、プラントのモータが先に止まり、その秒数後にスイッチが入る（FDI の遅れの模擬）。

飛行プロファイル（LOG を入れた時刻を 0 とする。実機ログリング 19.78 s に収める）:
    0–4 s   高度 PD で 1.0 m まで上昇（3 s ランプ）
    4–7 s   ホバー
    7 s     故障（--mode air）。--mode ground は 0 s（離陸前）にスイッチを入れる。--mode healthy は故障なし
    7–12 s  ホバー（回復を見る）
    12–14 s ロール +5°   14–16 s 中立
    16–18 s ピッチ +5°   18–19.5 s ホバー

操縦者（スロットル）: 高度 PD ＋ 積分 ＋ フィードフォワード。故障に気付いてから --pilot-delay 秒後に、
フィードフォワードを「FTC の釣り合い点」（担当 4 発が mg/4 を出す u_base）へ切り替える＝スロットルを足す。
"""
from __future__ import annotations

import argparse
import math
import time
from dataclasses import replace

import numpy as np

import rc
from nmpc_flight import (NmpcLoop, rc_base, with_sticks, LOG_COLS, thrust_inv, throttle_ratio_for_duty,
                         CH_FTC, SW_HIGH, SW_LOW)
from plant6dof import preset
from run_nmpc_eval import CONDS

MOTORS = ["FR", "FL", "MR", "ML", "RR", "RL"]
DIAG = [5, 4, 3, 2, 1, 0]
T_END = 19.5
T_FAULT = 7.0
U_DUTY_MAX = 0.95


def profile(t):
    """(h_ref [m], phi_ref [rad], theta_ref [rad])"""
    h = min(1.0, t / 3.0)
    phi = math.radians(5.0) if 12.0 <= t < 14.0 else 0.0
    th = math.radians(5.0) if 16.0 <= t < 18.0 else 0.0
    return h, phi, th


def ftc_u_base(mass, g=9.81):
    """FTC の釣り合い点: 担当 4 発が mg/4 を出す duty を、ファームの U_ref（担当発 = 1.5·u_base）で割った u_base。"""
    return thrust_inv(mass * g / 4.0) / 1.5


class FtcPilot:
    """スロットル = フィードフォワード + 高度 PD + 積分（実機に高度制御は無いので「上手な操縦者」の代わり）。"""

    def __init__(self, mass, kp=0.03, kd=0.04, ki=0.02, rate=0.6):
        self.ff_healthy = throttle_ratio_for_duty(thrust_inv(mass * 9.81 / 6.0))
        self.ff_ftc = throttle_ratio_for_duty(ftc_u_base(mass))
        self.ff = self.ff_healthy
        self.kp, self.kd, self.ki, self.rate = kp, kd, ki, rate
        self.i = 0.0
        self.r = 0.0

    def update(self, h_ref, h, vz, dt, airborne):
        e = h_ref - h
        if airborne:
            self.i = float(np.clip(self.i + self.ki * e * dt, -0.05, 0.05))
        tgt = self.ff + self.kp * e - self.kd * vz + self.i
        tgt = float(np.clip(tgt, 0.0, 0.40))
        dr = float(np.clip(tgt - self.r, -self.rate * dt, self.rate * dt))
        self.r += dr
        return self.r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mass", type=float, default=0.225)
    ap.add_argument("--motor", choices=MOTORS, default="FR")
    ap.add_argument("--mode", choices=["air", "ground", "healthy"], default="air")
    ap.add_argument("--detect-delay", type=float, default=0.0, help="モータが止まってから FTC スイッチが入るまで [s]")
    ap.add_argument("--pilot-delay", type=float, default=0.3, help="故障から操縦者がスロットルを足し始めるまで [s]")
    ap.add_argument("--no-pilot-ff", action="store_true", help="故障後もフィードフォワードを健全時のまま（積分だけで追う）")
    ap.add_argument("--iscale", default="1:1:1", help="プラントの真の慣性 / ファームの値（Ix:Iy:Iz）")
    ap.add_argument("--cond", choices=list(CONDS), default="shake")
    ap.add_argument("--wind", type=float, default=0.0, help="定常風 [m/s]（北向き）＋突風 σ 0.5")
    ap.add_argument("--yaw", default="1:1", help="プラントのヨー物理の倍率 Cr:C_QT（ファームの値に対する真値。例 3.7:0.69）")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    c = dict(CONDS[a.cond])
    mi = MOTORS.index(a.motor)
    sx, sy, sz = (float(v) for v in a.iscale.split(":"))

    pp = preset("full")
    pp = replace(pp, mass=a.mass, Ix=pp.Ix * sx, Iy=pp.Iy * sy, Iz=pp.Iz * sz, proc_sigma=c["proc"])
    ycr, yqt = (float(v) for v in a.yaw.split(":"))
    pp = replace(pp, Cr=pp.Cr * ycr, C_QT=pp.C_QT * yqt)
    if a.wind > 0:
        pp = replace(pp, wind_ned=(a.wind, 0.0, 0.0), gust_sigma=0.5, gust_tau=1.0)

    wall0 = time.time()
    cl = NmpcLoop(pp, seed=a.seed, gyro_sigma=c["gyro"], acc_sigma_mg=c["acc"],
                  out_delay_ticks=c["delay"], rebuild_hold_ticks=c["hold"], gyro_bias_rw=c.get("bias_rw", 0.0), log=True)
    cl.sil.set_ftc_target(mi)
    t_arm = cl.arm_nmpc()
    cl.rows.clear()
    pilot = FtcPilot(a.mass)
    t_motor_dead = {"air": T_FAULT, "ground": 0.0, "healthy": None}[a.mode]
    t_switch = None if t_motor_dead is None else t_motor_dead + a.detect_delay
    t_pilot = None if t_motor_dead is None else (t_motor_dead if a.mode == "ground" else t_motor_dead + a.pilot_delay)

    # プラントのモータを先に止める（detect-delay > 0 のとき）: NmpcLoop が渡す duty の該当発を 0 にする
    plant_step = cl.plant.step
    state = {"dead": False}

    def step_with_fault(duty):
        if state["dead"]:
            duty = list(duty); duty[mi] = 0.0
        return plant_step(duty)
    cl.plant.step = step_with_fault

    t0 = cl.t
    crash = ""
    airborne = False
    refs = []
    r = 0.0
    n = int(T_END / 0.0025)
    for k in range(n):
        tt = cl.t - t0
        if t_motor_dead is not None and tt >= t_motor_dead:
            state["dead"] = True
        ftc_on = t_switch is not None and tt >= t_switch
        if t_pilot is not None and tt >= t_pilot and not a.no_pilot_ff:
            pilot.ff = pilot.ff_ftc
        h_ref, phi_ref, th_ref = profile(tt)
        if k % 8 == 0:   # 操縦者の更新 50 Hz
            r = pilot.update(h_ref, cl.plant.altitude, cl.plant.climb_rate, 0.02, airborne)
        base = rc_base(log_on=True, ftc_on=ftc_on)
        cl.set_rc(rc.with_throttle(with_sticks(base, phi_ref, th_ref), r))
        cl.step()
        refs.append((tt, h_ref, phi_ref, th_ref, r, 1.0 if ftc_on else 0.0, 1.0 if state["dead"] else 0.0))
        pl = cl.plant
        if pl.altitude > 0.1:
            airborne = True
        if abs(pl.phi) > math.radians(70) or abs(pl.theta) > math.radians(70):
            crash = f"姿勢 70° 超 t={tt:.2f}"
        elif airborne and tt > 4.0 and pl.altitude < 0.02:
            crash = f"接地 t={tt:.2f}"
        if crash:
            break

    st = cl.sil.status()
    L = cl.logdata()
    keys = list(cl.rows[0].keys())
    P = np.array([[row[kk] for kk in keys] for row in cl.rows], dtype=float)
    R = np.array(refs, dtype=float)
    np.savez_compressed(a.out, plant=P, plant_cols=np.array(keys), log=L, log_cols=np.array(LOG_COLS),
                        refs=R, refs_cols=np.array(["t", "h_ref", "phi_ref", "theta_ref", "thr", "ftc_sw", "motor_dead"]),
                        crash=crash, cond=a.cond, mass=a.mass, motor=a.motor, mode=a.mode, detect_delay=a.detect_delay,
                        pilot_delay=a.pilot_delay, pilot_ff=not a.no_pilot_ff, iscale=a.iscale, yaw=a.yaw, wind=a.wind, seed=a.seed,
                        status=np.array(list(st.values())), fs_cause=st['fs_cause'], n_holds=cl.n_holds,
                        order_checks=cl.order_checks, order_bad=cl.order_bad, t_arm=t_arm, wall=time.time() - wall0)
    print(f"m={a.mass} {a.motor} {a.mode} dd={a.detect_delay} I={a.iscale} Y={a.yaw} w={a.wind} {a.cond} s{a.seed}: "
          f"crash='{crash}' fs={st['fs']}/{st['fs_cause']} log={L.shape} order_bad={cl.order_bad}/{cl.order_checks} "
          f"wall={time.time() - wall0:.1f}s")


if __name__ == "__main__":
    main()
