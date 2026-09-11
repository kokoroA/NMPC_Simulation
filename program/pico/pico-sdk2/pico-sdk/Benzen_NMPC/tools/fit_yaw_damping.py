#!/usr/bin/env python3
"""ヨー軸吊り試験の解析 — C_QT/Iz, tau_f/Iz, b/Iz, Cr/Iz を求める。

対象ログ: 機内 100 Hz リングログ (log_output() が吐く CSV)
出典     : docs/20260905_yaw_damping_experiment_protocol.md
理屈     : docs/ai/decisions/20260905_cr_upper_bound_and_yaw_hang_test.md

使い方:
    # 自由減衰区間 (フェーズ4) から減衰係数を出す
    python3 fit_yaw_damping.py yaw_test.csv --decay 12.0 25.0

    # ステップ区間 (フェーズ2) から入力ゲイン C_QT/Iz を出す
    python3 fit_yaw_damping.py yaw_test.csv --step 5.0 5.5 --u0 0.60 --delta 0.05

    # ハードウェア無しで手法を検証する
    python3 fit_yaw_damping.py --selftest
"""
import argparse
import math
import sys

import numpy as np

D2R = math.pi / 180.0
R2D = 180.0 / math.pi

# 実測静推力フィット T(u) [N]  (docs/thrust_sysid_f1404.md)
THRUST = {
    "GEMFAN-3630-3": (-0.2265, 1.0525, 0.0452),
    "betaFPV-D90s-3": (0.0904, 0.5431, 0.0243),
}
# ヨー反トルクの符号 [FR, FL, MR, ML, RR, RL]  (nmpc_model.cpp の kS)
KS = np.array([-1.0, 1.0, 1.0, -1.0, -1.0, 1.0])


def thrust(u, prop):
    a, b, c = THRUST[prop]
    u = np.clip(u, 0.0, 1.0)
    return a * u * u + b * u + c


def load_log(path):
    """機内 100 Hz ログを読む。#cols 行から列名を取る。"""
    raw = open(path, errors="replace").read().splitlines()
    cols = [l for l in raw if l.startswith("#cols:")]
    if not cols:
        sys.exit(f"{path}: '#cols:' 行が無い。機内 100Hz ログではない")
    cols = cols[0].replace("#cols:", "").strip().split(",")
    rows = []
    for l in raw:
        if not l or l.startswith("#"):
            continue
        p = l.split(",")
        if len(p) != len(cols):
            continue
        try:
            rows.append([float(x) for x in p])
        except ValueError:
            continue
    a = np.array(rows)
    return {c: a[:, i] for i, c in enumerate(cols)}


def fit_decay(t, r, window_s=0.5, overlap=0.5, use_friction=True, use_linear=True):
    """自由減衰 (tau_z = 0) から  rdot = -(tau_f + b r + Cr r|r|)/Iz  の係数比を出す。

    微分せず、窓 [t0,t1] で積分した形

        r(t1) - r(t0) = -(tau_f/Iz)(t1-t0) - (b/Iz)∫r dt - (Cr/Iz)∫r|r| dt

    を並べて最小二乗する。戻り値は (tau_f/Iz, b/Iz, Cr/Iz, cond, 残差rms)。
    使わない項は None を返す。
    """
    dt = float(np.median(np.diff(t)))
    w = max(int(round(window_s / dt)), 8)
    step = max(int(w * (1.0 - overlap)), 1)
    A, y = [], []
    for s in range(0, len(t) - w, step):
        e = s + w
        seg_t, seg_r = t[s:e + 1], r[s:e + 1]
        row = []
        if use_friction:
            row.append(-(seg_t[-1] - seg_t[0]))
        if use_linear:
            row.append(-np.trapezoid(seg_r, seg_t))
        row.append(-np.trapezoid(seg_r * np.abs(seg_r), seg_t))
        A.append(row)
        y.append(seg_r[-1] - seg_r[0])
    A, y = np.array(A), np.array(y)
    if len(y) < 4:
        sys.exit("窓が足りない。減衰区間を長く取るか window_s を小さくする")
    sol, *_ = np.linalg.lstsq(A, y, rcond=None)
    res = float(np.sqrt(np.mean((A @ sol - y) ** 2)))
    k = 0
    tau_f = sol[k] if use_friction else None
    if use_friction:
        k += 1
    b = sol[k] if use_linear else None
    if use_linear:
        k += 1
    return tau_f, b, sol[k], float(np.linalg.cond(A)), res


def report_decay(t, r, iz=None, label=""):
    tf, b, cr, cond, res = fit_decay(t, r)
    print(f"\n=== 自由減衰フィット {label} ===")
    print(f"  区間 {t[0]:.2f}-{t[-1]:.2f} s ({len(t)} 点)、"
          f"|r| {abs(r).max()*R2D:.0f} → {abs(r[-1])*R2D:.0f} deg/s"
          f"  (掃引比 {abs(r).max()/max(abs(r[-1]),1e-6):.1f}:1)")
    print(f"  tau_f/Iz = {tf:+.4f} 1/s^2   [クーロン摩擦。軸受由来]")
    print(f"  b/Iz     = {b:+.4f} 1/s      [線形減衰。ロータ H 力など]")
    print(f"  Cr/Iz    = {cr:+.4f} 1/rad   [二乗減衰。★これが欲しい量★]")
    print(f"  条件数 {cond:.1f}  (10 前後なら良好。50 超なら掃引範囲不足)")
    print(f"  残差 rms {res*R2D:.3f} deg/s")
    if iz:
        print(f"  Iz={iz:.3e} を掛けると:  tau_f={tf*iz:.3e} Nm   b={b*iz:.3e} Nms/rad   "
              f"Cr={cr*iz:.3e} Nms^2/rad^2")
    if cr <= 0:
        print("  ⚠ Cr が非正。掃引範囲不足か、摩擦が支配的。フェーズ0 のベースラインを確認すること")
    if cond > 50:
        print("  ⚠ 条件数が大きい。r の掃引範囲を広げる (初速を上げる / 減衰を長く取る)")
    return tf, b, cr


def report_step(t, r, u, prop, iz=None):
    """フェーズ2: r=0 近傍の立ち上がりから C_QT/Iz を出す。"""
    tz_over_cqt = (thrust(u, prop) * KS).sum(axis=1)   # tau_z = C_QT * この値
    slope = np.polyfit(t, r, 1)[0]                     # rdot [rad/s^2]
    m = float(np.mean(tz_over_cqt))
    print(f"\n=== 立ち上がりフィット (C_QT/Iz) ===")
    print(f"  区間 {t[0]:.2f}-{t[-1]:.2f} s、r {r[0]*R2D:+.1f} → {r[-1]*R2D:+.1f} deg/s")
    print(f"  rdot = {slope*R2D:.1f} deg/s^2")
    print(f"  Sum s_i T(u_i) = {m:.4f} N")
    print(f"  C_QT/Iz = rdot / Sum = {slope/m:.3f} 1/(N m)... 単位は [1/(kg m^2)] 換算")
    if iz:
        print(f"  Iz={iz:.3e} → C_QT = {slope/m*iz:.5f} m   (推定値 0.01113 と比較)")
    return slope / m


def selftest():
    print("=== セルフテスト: 既知の真値を合成データから復元できるか ===")
    iz, tau_f, b, cr = 3.00e-3, 5.0e-5, 2.0e-5, 1.0e-4
    print(f"  真値  tau_f={tau_f:.2e}  b={b:.2e}  Cr={cr:.2e}  Iz={iz:.2e}")
    rng = np.random.default_rng(0)
    dt, T = 1e-4, 25.0
    n = int(T / dt)
    r = np.zeros(n)
    r[0] = 200 * D2R
    for k in range(n - 1):
        if r[k] <= 1e-4:
            r[k + 1:] = 0.0
            break
        r[k + 1] = r[k] - (tau_f + b * r[k] + cr * r[k] * abs(r[k])) / iz * dt
    idx = np.arange(0, n, 100)
    keep = r[idx] > 1e-4
    ts = (idx * dt)[keep]
    rs = r[idx][keep] + rng.normal(0, 0.5 * D2R, keep.sum())   # ジャイロノイズ 0.5 deg/s
    tf_h, b_h, cr_h = report_decay(ts, rs, iz=iz, label="(合成データ)")
    err = abs(cr_h * iz - cr) / cr * 100
    print(f"\n  Cr 復元誤差 {err:.1f} %  →  {'OK' if err < 15 else '要確認'}")
    return 0 if err < 15 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="?", help="機内 100Hz ログ")
    ap.add_argument("--decay", nargs=2, type=float, metavar=("T0", "T1"),
                    help="自由減衰区間 [s] (フェーズ4)")
    ap.add_argument("--step", nargs=2, type=float, metavar=("T0", "T1"),
                    help="ステップ立ち上がり区間 [s] (フェーズ2)")
    ap.add_argument("--baseline", nargs=2, type=float, metavar=("T0", "T1"),
                    help="モータ OFF のベースライン減衰区間 [s] (フェーズ0)")
    ap.add_argument("--iz", type=float, help="Iz [kg m^2]。与えると絶対値も表示する")
    ap.add_argument("--prop", default="GEMFAN-3630-3", choices=list(THRUST))
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())
    if not a.csv:
        ap.error("csv か --selftest が必要")

    log = load_log(a.csv)
    t, r = log["t"], log["r"]
    U = np.vstack([log[f"u_{s}"] for s in ("fr", "fl", "mr", "ml", "rr", "rl")]).T

    if a.baseline:
        m = (t >= a.baseline[0]) & (t <= a.baseline[1])
        print("\n#### フェーズ0: 軸受摩擦 + 機体空力のベースライン (モータ OFF) ####")
        report_decay(t[m], r[m], iz=a.iz, label="(ベースライン)")
    if a.step:
        m = (t >= a.step[0]) & (t <= a.step[1])
        report_step(t[m], r[m], U[m], a.prop, iz=a.iz)
    if a.decay:
        m = (t >= a.decay[0]) & (t <= a.decay[1])
        print("\n#### フェーズ4: ロータ回転中の自由減衰 ####")
        report_decay(t[m], r[m], iz=a.iz, label="(本計測)")
        print("\n  ロータ由来の減衰 = 本計測 - ベースライン。両方を取ってから差を取ること。")
    if not (a.decay or a.step or a.baseline):
        ap.error("--decay / --step / --baseline のいずれかを指定")


if __name__ == "__main__":
    main()
