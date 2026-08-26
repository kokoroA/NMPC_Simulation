#!/usr/bin/env python3
"""M5 デモ: 自動機動 3 種を連続で飛ばしてログと図を出す。

    python3 run_m5_demo.py [--throttle 0.13]
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logger
import plots
import rc
from sim import ClosedLoop


def run_mode(cl, mode, label, max_s=45.0):
    print(f"  {label}...")
    cl.sil.set_flight_mode(mode)
    n = 0
    while n * 0.0025 < max_s:
        cl.run(40)
        n += 40
        if cl.sil.flight_mode == 0 and n > 400:
            break
    cl.run_seconds(3.0)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--throttle", type=float, default=0.13)
    ap.add_argument("--csv", action="store_true")
    args = ap.parse_args()

    print(f"=== M5 デモ: 自動機動 3 種 (throttle={args.throttle}) ===")
    cl = ClosedLoop()
    cl.arm()
    cl.set_rc(rc.with_throttle(rc.neutral(), args.throttle))
    cl.run_seconds(3.0)

    run_mode(cl, cl.sil.MODE_AUTO_HORIZONTAL_TURN, "自動水平旋回")
    run_mode(cl, cl.sil.MODE_AUTO_FIGURE_EIGHT, "自動 8 の字")
    cl.plant.pos[:] = 0.0
    cl.plant.vel[:] = 0.0
    run_mode(cl, cl.sil.MODE_AUTO_CLIMBING_TURN, "自動上昇旋回")

    run_dir = cl.save_run(note="M5: 自動水平旋回 → 8の字 → 上昇旋回", csv=args.csv)
    print(f"\n保存: {run_dir}  ({len(cl.rows)} 行 / {cl.t:.1f} s)")

    data, meta = logger.load(run_dir)
    print()
    print(plots.summarize(data, meta))
    made = [plots.plot_overview(data, meta, run_dir / "overview.png"),
            plots.plot_saturation(data, run_dir / "saturation.png")]
    t3 = plots.plot_trajectory3d(data, run_dir / "trajectory.png")
    if t3:
        made.append(t3)
    print("\n生成:")
    for p in made:
        print(f"  {p}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
