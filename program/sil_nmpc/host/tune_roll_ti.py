#!/usr/bin/env python3
"""ロール角度 PID の Ti を振って、旋回中のバンク達成率を調べる。

M5 で「指令 18deg に対し実 8.4deg（47%）」だった原因は、旋回の運動学的
カップリングをロール軸の積分が打ち消せていないこと。Ti をいくつにすれば
効くのかを SIL で実測する。

    python3 tune_roll_ti.py            # 既定の候補を掃引
    python3 tune_roll_ti.py 5 2 1      # Ti を指定

⚠ ファームはグローバル状態を持つので 1 プロセス 1 条件。
   このスクリプトは条件ごとに子プロセスを起動する。
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_TI = [100000.0, 10000.0, 20.0, 10.0, 5.0, 2.0, 1.0]

CHILD = r'''
import sys; sys.path.insert(0, {here!r})
import numpy as np, rc
from sim import ClosedLoop
DEG = np.pi/180
ti = float(sys.argv[1])
cl = ClosedLoop(log=False)
cl.arm()
if ti > 0:
    cl.sil.set_pid(cl.sil.PID_PHI, 1.0, ti, 0.0, 0.125, 0.01)
cl.set_rc(rc.with_throttle(rc.neutral(), 0.13))
cl.run_seconds(2.0)
cl.sil.set_flight_mode(cl.sil.MODE_AUTO_HORIZONTAL_TURN)
samples = []
n = 0
while n < 16000:
    cl.run(40); n += 40
    st = cl.sil.state()
    if abs(st["Rref"]) > 0.9*35*DEG:          # 定常旋回区間だけ拾う
        samples.append((st["phi_ref"], cl.plant.phi, cl.plant.theta))
    if cl.sil.flight_mode == 0 and n > 400:
        break
a = np.array(samples)
if len(a) < 10:
    print("NODATA"); sys.exit(0)
half = len(a)//2                              # 後半＝定常
ref, act, th = a[half:,0].mean(), a[half:,1].mean(), a[half:,2].mean()
print("%.6f %.6f %.6f" % (ref/DEG, act/DEG, th/DEG))
'''


def run_one(ti: float):
    out = subprocess.run(
        [sys.executable, "-c", CHILD.format(here=str(HERE)), str(ti)],
        capture_output=True, text=True, timeout=300)
    line = out.stdout.strip().splitlines()[-1] if out.stdout.strip() else ""
    if not line or line == "NODATA":
        return None
    try:
        return tuple(float(x) for x in line.split())
    except ValueError:
        return None


def main() -> int:
    tis = [float(x) for x in sys.argv[1:]] or DEFAULT_TI
    print("ロール角度 PID の Ti 掃引（水平旋回・定常区間の平均）")
    print("Kp=1.0 固定。実機の現行値は control.cpp:699 を参照。\n")
    print("      Ti      指令バンク    実バンク   達成率    ピッチ")
    print("  " + "-" * 54)
    for ti in tis:
        r = run_one(ti)
        if r is None:
            print(f"  {ti:>9.1f}   （データなし）")
            continue
        ref, act, th = r
        print(f"  {ti:>9.1f}   {ref:>8.2f}deg  {act:>8.2f}deg  {100*act/ref:>5.0f} %  {th:>8.2f}deg")
    print()
    print("  Ti は積分時間 [s]。pid.cpp: integral += h*(err+err_prev)/2/Ti")
    print("  角度ループは h=0.01s。Ti が旋回時間(約10s)より桁違いに大きいと積分は効かない。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
