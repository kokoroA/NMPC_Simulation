#!/usr/bin/env python3
"""M3 デモ: 一連の飛行シーケンスを飛ばしてログを保存する。

    python3 run_m3_demo.py [--csv] [--throttle 0.30]

保存後に plots.py で描画すると overview / saturation / trajectory が出る。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import firmware_consts as fc
import plots
import rc
from sim import ClosedLoop

DEG = np.pi / 180.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--throttle", type=float, default=0.30,
                    help=f"使える帯は {fc.THROTTLE_MOTORS_ON:.2f}〜{fc.THROTTLE_DUTY_SATURATES:.2f}")
    ap.add_argument("--csv", action="store_true")
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()

    thr = args.throttle
    print(f"=== M3 デモ飛行 (throttle={thr}) ===")
    cl = ClosedLoop()

    print("  アーム中（ウォームアップ 6000 tick + lock_com ジェスチャ）...")
    cl.arm()
    base = rc.with_throttle(rc.neutral(), thr)
    cl.set_rc(base)
    cl.run_seconds(3.0)

    print("  ロール ±10deg ステップ...")
    for target in (+10, -10, 0):
        cl.set_rc(rc.with_roll_ref(base, target * DEG))
        cl.run_seconds(3.0)

    print("  ピッチ ±10deg ステップ...")
    for target in (+10, -10, 0):
        cl.set_rc(rc.with_pitch_ref(base, target * DEG))
        cl.run_seconds(3.0)

    print("  複合舵（ロール+8 / ピッチ-6）...")
    cl.set_rc(rc.with_pitch_ref(rc.with_roll_ref(base, 8 * DEG), -6 * DEG))
    cl.run_seconds(4.0)
    cl.set_rc(base)
    cl.run_seconds(3.0)

    print("  突風外乱（p=+120, q=-80 deg/s）...")
    cl.disturb_rates(dp=+120 * DEG, dq=-80 * DEG)
    cl.run_seconds(5.0)

    print("  スロットル掃引...")
    for r in (thr, thr + 0.06, thr - 0.06, thr):
        cl.set_rc(rc.with_throttle(rc.neutral(), r))
        cl.run_seconds(2.0)

    run_dir = cl.save_run(note="M3 デモ: ステップ・複合舵・突風・スロットル掃引", csv=args.csv)
    print(f"\n保存: {run_dir}")
    print(f"  行数 {len(cl.rows)}  シミュ時間 {cl.t:.2f} s")

    if not args.no_plot:
        import logger
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
