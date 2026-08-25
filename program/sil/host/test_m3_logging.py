#!/usr/bin/env python3
"""M3: ログと描画の検証。

検証内容:
  1. 全状態が欠落なく記録される（400 Hz で行が落ちない）
  2. 保存 -> 読み戻しで値が一致する（往復の健全性）
  3. meta に再現に必要な情報が揃う（ファームの git hash 等）
  4. 図が生成でき、日本語が豆腐にならない
"""
from __future__ import annotations

import sys
import tempfile
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import logger
import plots
import rc
from sim import DT, ClosedLoop

DEG = np.pi / 180.0
failures = []

REQUIRED_COLS = {
    "t", "phi_true", "theta_true", "psi_true", "p_true", "q_true", "r_true",
    "phi_est", "theta_est", "phi_ref", "theta_ref",
    "Pref", "Qref", "Rref", "P_com", "Q_com", "R_com",
    "T_ref", "arm", "mode", "alt", "pos_n", "pos_e",
    "ch_throttle", "ch_aileron", "ch_elevator", "ch_rudder",
    "duty_fr", "duty_fl", "duty_mr", "duty_ml", "duty_rr", "duty_rl",
    "host_core0_ns", "host_core1_ns",
}
REQUIRED_META = {"note", "throttle", "firmware_source_sha256", "sil_git",
                 "plant_params", "firmware_consts", "rows", "created"}


def check(cond, msg, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}{('  ' + extra) if extra else ''}")
    if not cond:
        failures.append(msg)


def main() -> int:
    print("=== M3: ログと描画の検証 ===\n")

    print("--- 1. 記録の完全性 ---")
    cl = ClosedLoop()
    cl.arm()
    base = rc.with_throttle(rc.neutral(), 0.30)
    cl.set_rc(base)
    cl.run_seconds(2.0)
    cl.set_rc(rc.with_roll_ref(base, 10 * DEG))
    cl.run_seconds(2.0)
    cl.set_rc(base)
    cl.run_seconds(1.0)

    n = len(cl.rows)
    expected = int(round(cl.t / DT))
    check(abs(n - expected) <= 1, "行数が tick 数と一致（取りこぼしなし）",
          f"{n} 行 / 期待 {expected}")

    cols = set(cl.rows[-1].keys())
    missing = REQUIRED_COLS - cols
    check(not missing, "必要な列が全て存在", f"{len(cols)} 列" + (f" 欠落={missing}" if missing else ""))

    rate = n / cl.t
    check(abs(rate - 400.0) < 1.0, "記録レートが 400 Hz", f"{rate:.1f} Hz")

    print("\n--- 2. 保存 -> 読み戻しの往復 ---")
    with tempfile.TemporaryDirectory() as td:
        run_dir = cl.save_run(note="M3 テスト", name="run_test", csv=True)
        # 既定の runs/ に保存されるので、そこから読む
        data, meta = logger.load(run_dir)

        check(len(data["t"]) == n, "行数が保存前後で一致", f"{len(data['t'])} == {n}")
        same = True
        for key in ("phi_true", "theta_true", "duty_fr", "T_ref"):
            orig = np.array([r[key] for r in cl.rows], dtype=np.float32)
            if not np.allclose(orig, data[key], rtol=0, atol=0):
                same = False
                print(f"      不一致: {key}")
        check(same, "float32 で値が完全一致（往復で壊れない）")

        check((run_dir / "data.csv").exists(), "CSV も出力される")
        npz_kb = (run_dir / "data.npz").stat().st_size / 1024
        csv_kb = (run_dir / "data.csv").stat().st_size / 1024
        check(npz_kb < csv_kb, "npz が CSV より小さい",
              f"npz {npz_kb:.0f} KB < csv {csv_kb:.0f} KB ({csv_kb/npz_kb:.1f}x)")

        print("\n--- 3. meta の再現情報 ---")
        missing_meta = REQUIRED_META - set(meta.keys())
        check(not missing_meta, "meta に必要なキーが揃う",
              f"欠落={missing_meta}" if missing_meta else "")
        check(bool(meta.get("firmware_source_sha256")),
              "ファームのソース内容ハッシュが記録される（未追跡のため git hash では特定できない）",
              f"{meta.get('firmware_source_sha256')} / {meta.get('firmware_source_files')} ファイル")
        check(isinstance(meta.get("throttle"), float) and 0.0 <= meta["throttle"] <= 1.0,
              "throttle が比率(0..1)で記録される", str(meta.get("throttle")))
        check("T_max" in meta.get("plant_params", {}), "機体パラメータが記録される",
              f"T_max={meta['plant_params'].get('T_max')}")

        print("\n--- 4. 描画 ---")
        ov = plots.plot_overview(data, meta, run_dir / "overview.png")
        sat = plots.plot_saturation(data, run_dir / "saturation.png")
        check(ov.exists() and ov.stat().st_size > 10_000, "overview.png を生成",
              f"{ov.stat().st_size/1024:.0f} KB")
        check(sat.exists(), "saturation.png を生成")
        font = plots._use_japanese_font()
        check(font is not None, "日本語フォントが使える（図が豆腐にならない）", str(font))

        print("\n--- 5. サマリ ---")
        print("\n".join("  " + l for l in plots.summarize(data, meta).splitlines()))

    print("\n" + "=" * 52)
    if failures:
        print(f"FAILED: {len(failures)} 件")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("M3 全項目 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
