"""Benzen_NMPC（無改造ファーム）を SIL で 1 回飛ばして npz に保存する（2026-10-08）。

1 プロセス 1 インスタンス（SIL の制約）なので、条件ごとに別プロセスで呼ぶこと。

    python3 run_nmpc_eval.py --cond real --seed 0 --out ../results/20261008/real_s0.npz

飛行プロファイル（LOG を入れた時刻を 0 とする。実機ログリング 19.78 s に収める）:
    0–4 s   高度 PD で 1.0 m まで上昇（3 s ランプ）
    4–7 s   ホバー
    7–9 s   ロール +15°   9–11 s 中立
    11–13 s ピッチ +10°   13–15 s 中立
    15–19.5 s ホバー
"""
from __future__ import annotations

import argparse
import math
import time

import numpy as np

import rc
from nmpc_flight import NmpcLoop, RealPlantParams, Pilot, rc_base, with_sticks, LOG_COLS

# 条件: 実機の測定値に合わせたもの
#   gyro σ 0.03 rad/s, 機体外乱 σ 0.5 rad/s/√s → 静かなホバーの p rms を実機 7–8.6 deg/s に合わせた値
#     （docs/ai/decisions/20261006_sim_flies_real_crashes_gap_is_in_controller_internals_not_physics.md）
#   加速度 σ 60 mg  → 実機 acc_norm の std 0.67–0.90 m/s² 相当
#   出力遅延 4 tick = 10 ms → 実機 loop_us 平均 9.8–10.3 ms
#   再構築据え置き 12 tick = 30 ms → 実機 loop_us 最大 31–45 ms（フル再構築周期）
CONDS = {
    "ideal": dict(gyro=0.0, acc=0.0, proc=0.0, delay=0, hold=0),
    "delay": dict(gyro=0.0, acc=0.0, proc=0.0, delay=4, hold=12),
    "noise": dict(gyro=0.03, acc=60.0, proc=0.5, delay=0, hold=0),
    "real": dict(gyro=0.03, acc=60.0, proc=0.5, delay=4, hold=12),
    # アブレーション比較用: 再構築据え置きなし（毎周期再構築のビルドでは据え置きが毎周期になってしまうため）
    "real_nh": dict(gyro=0.03, acc=60.0, proc=0.5, delay=4, hold=0),
    # 2026-10-08: 6 自由度プラント（full）で実機ログの揺れに較正した値（results/20261008_shake/calib.sh の候補 A）
    #   静かな区間の p/q/r rms 7.2/4.9/7.1 deg/s（実機 3 便平均 7.0/4.8/7.9）、加速度ノルム std 0.73（0.74）、
    #   EKF 補正量 ekf_dphi/dtheta 0.83/0.80 deg/s（0.71/1.08）
    "shake": dict(gyro=0.02, acc=75.0, proc=(0.35, 0.22, 0.30), delay=4, hold=12),
    # shake + ジャイロバイアスのランダムウォーク 0.001 rad/s/√s。EKF 対策の比較用
    #   （0.003 だと離陸前に EKF 残差が 1 deg/s を超え、ファームの離陸ゲートが 12 本中 3 本で離陸を拒否した。実機ではゲートは毎回開く）
    "shake_b": dict(gyro=0.02, acc=75.0, proc=(0.35, 0.22, 0.30), delay=4, hold=12, bias_rw=0.001),
}

T_END = 19.5

# ---- プラントの段階（2026-10-08）: 下へ行くほど 1 要素ずつ足す ----
from dataclasses import replace as _replace
from plant6dof import preset as _preset, Plant6Params
_FULL = _preset("full")
PLANTS = {
    "old": None,                                                     # 旧 RealPlant（姿勢のみ・並進は開ループ）
    "legacy": _preset("legacy"),                                     # 6 自由度だが旧と同じ扱い（回帰確認）
    "sf_nodrag": _preset("rigid"),                                   # 説明用の極端: 比力加速度計だが抗力ゼロ（非物理）
    "sf": _replace(_preset("rigid"), C_drag=_FULL.C_drag, k_rotor_drag=_FULL.k_rotor_drag),     # 比力加速度計 + 形状抗力・ロータ抗力
    "bf": _replace(_preset("rigid"), C_drag=_FULL.C_drag, k_rotor_drag=_FULL.k_rotor_drag,
                   k_bf_roll=_FULL.k_bf_roll, k_bf_pitch=_FULL.k_bf_pitch),                    # + ブローバック
    "motor": _replace(_FULL, ge_enable=False, v_pitch_speed=0.0),     # + 回転数モデル・ロータ慣性
    "full": _FULL,                                                   # + 地面効果・軸流
    "wind": _replace(_FULL, wind_ned=(1.5, 0.0, 0.0), gust_sigma=0.5, gust_tau=1.0),  # + 風 1.5 m/s・突風 0.5 m/s
}


def _parse_set(items):
    out = {}
    for it in items or []:
        k, v = it.split("=", 1)
        if ":" in v:                                # タプルは 0.3:0.2:0.25 と書く（xargs/シェルで括弧が壊れるため）
            out[k] = tuple(float(x) for x in v.split(":"))
        else:
            out[k] = eval(v, {"__builtins__": {}})  # 数値のみ想定（手元実験用）
    return out


def profile(t):
    """(h_ref [m], phi_ref [rad], theta_ref [rad])"""
    h = min(1.0, t / 3.0)
    phi = math.radians(15.0) if 7.0 <= t < 9.0 else 0.0
    th = math.radians(10.0) if 11.0 <= t < 13.0 else 0.0
    return h, phi, th


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cond", choices=list(CONDS), required=True)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", required=True)
    ap.add_argument("--legacy-order", action="store_true", help="修正前 SIL のモータ順でプラントへ渡す（不具合の再現）")
    ap.add_argument("--plant", choices=list(PLANTS), default="old", help="プラントの段階（既定: 旧プラント）")
    ap.add_argument("--set", nargs="*", help="Plant6Params の上書き（例: k_bf_roll=0.0 proc_sigma=0.0）")
    ap.add_argument("--gyro", type=float, default=None, help="ジャイロ雑音 σ [rad/s] の上書き")
    ap.add_argument("--acc", type=float, default=None, help="加速度計雑音 σ [mg] の上書き")
    a = ap.parse_args()
    c = dict(CONDS[a.cond])
    if a.gyro is not None: c["gyro"] = a.gyro
    if a.acc is not None: c["acc"] = a.acc

    wall0 = time.time()
    if PLANTS[a.plant] is None:
        pp = RealPlantParams(proc_sigma=c["proc"])
    else:
        kw = dict(proc_sigma=c["proc"]); kw.update(_parse_set(a.set))     # --set が条件の値より優先
        pp = _replace(PLANTS[a.plant], **kw)
    cl = NmpcLoop(pp, seed=a.seed, gyro_sigma=c["gyro"], acc_sigma_mg=c["acc"],
                  out_delay_ticks=c["delay"], rebuild_hold_ticks=c["hold"], gyro_bias_rw=c.get("bias_rw", 0.0),
                  legacy_order=a.legacy_order, log=True)
    t_arm = cl.arm_nmpc()
    cl.rows.clear()
    pilot = Pilot(pp.mass)
    base = rc_base(log_on=True)
    t0 = cl.t
    crash = ""
    airborne = False
    refs = []
    n = int(T_END / 0.0025)
    for k in range(n):
        tt = cl.t - t0
        h_ref, phi_ref, th_ref = profile(tt)
        if k % 8 == 0:   # 操縦者の更新 50 Hz
            r = pilot.update(h_ref, cl.plant.altitude, cl.plant.climb_rate, 0.02)
        cl.set_rc(rc.with_throttle(with_sticks(base, phi_ref, th_ref), r))
        cl.step()
        refs.append((tt, h_ref, phi_ref, th_ref, r))
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
                        refs=R, crash=crash, cond=a.cond, plant_kind=a.plant, plant_set=" ".join(a.set or []), seed=a.seed, status=np.array(list(st.values())),
                        n_holds=cl.n_holds, gyro=c["gyro"], acc=c["acc"], order_checks=cl.order_checks, order_bad=cl.order_bad, t_arm=t_arm, wall=time.time() - wall0)
    print(f"{a.plant}/{a.cond} s{a.seed}: crash='{crash}' fs={st['fs']}/{st['fs_cause']} log={L.shape} holds={cl.n_holds} order_bad={cl.order_bad}/{cl.order_checks} "
          f"wall={time.time() - wall0:.1f}s")


if __name__ == "__main__":
    main()
