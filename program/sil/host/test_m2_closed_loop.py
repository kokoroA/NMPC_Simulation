#!/usr/bin/env python3
"""M2: 閉ループが成立することの検証。

プラント <-> 実機ファーム を結線し、姿勢制御が実際に働くことを確認する。

検証内容:
  1. アーム後、ホバー付近で姿勢が水平を保つ（発散しない）
  2. ロール/ピッチのステップ指令に PID が収束する
  3. 初期姿勢外乱から水平へ復帰する
  4. 推力収支の実測（実機パラメータの整合確認）
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rc
from plant import PlantParams
from sim import ClosedLoop

DEG = np.pi / 180.0
THROTTLE = 0.30          # Disable_duty(0.1) を超える最小限より少し上
failures = []


def check(cond, msg, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}{('  ' + extra) if extra else ''}")
    if not cond:
        failures.append(msg)


def peak_abs(cl, key, since):
    return max(abs(r[key]) for r in cl.rows[since:]) if len(cl.rows) > since else 0.0


def main() -> int:
    print("=== M2: 閉ループ検証 ===\n")

    print("--- 0. 推力収支（実機パラメータの整合確認）---")
    p = PlantParams()
    t_ref_at = lambda ratio: 0.7 * 7.4 * ratio
    duty_at = lambda ratio: t_ref_at(ratio) * 0.6
    # duty = 0.7*7.4*ratio*0.6 なので解析的に解ける
    hover_ratio = p.hover_duty / (0.7 * 7.4 * 0.6)
    min_flyable_ratio = 0.74 / (0.7 * 7.4)     # T_ref/BATTERY >= Disable_duty
    print(f"  ホバー必要 duty     = {p.hover_duty:.4f}  (T_max={p.T_max} N)")
    print(f"  それに要るスロットル = {hover_ratio:.4f}")
    print(f"  モータが回る最小     = {min_flyable_ratio:.4f}  -> duty {duty_at(min_flyable_ratio):.4f}")
    reachable = hover_ratio >= min_flyable_ratio
    if not reachable:
        print(f"  ⚠ ホバー点がモータ始動閾値より下。T_max={p.T_max} N は過大の疑い。")
        implied = p.mass * p.gravity / (6.0 * duty_at(min_flyable_ratio))
        print(f"    最小飛行 duty でホバーする T_max = {implied:.3f} N")

    print("\n--- 1. アームと水平保持 ---")
    cl = ClosedLoop()
    cl.arm()
    check(cl.sil.arm_flag == 2, "アーム成功")
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    mark = len(cl.rows)
    cl.run_seconds(5.0)
    ph, th = cl.plant.phi / DEG, cl.plant.theta / DEG
    check(abs(ph) < 2.0 and abs(th) < 2.0, "5 秒間 水平を保持",
          f"phi={ph:+.3f} theta={th:+.3f} deg")
    check(peak_abs(cl, "phi_true", mark) / DEG < 5.0, "過渡でも 5deg 以内",
          f"peak={peak_abs(cl,'phi_true',mark)/DEG:.3f} deg")

    print("\n--- 2. ロール ステップ応答 ---")
    for target_deg in (+10.0, -10.0):
        cl.set_rc(rc.with_roll_ref(rc.with_throttle(rc.neutral(), THROTTLE), target_deg * DEG))
        cl.run_seconds(4.0)
        got = cl.plant.phi / DEG
        err = got - target_deg
        check(abs(err) < 2.0, f"ロール {target_deg:+.0f}deg に収束",
              f"真値={got:+.3f} 誤差={err:+.3f} deg")
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    cl.run_seconds(3.0)

    print("\n--- 3. ピッチ ステップ応答 ---")
    for target_deg in (+10.0, -10.0):
        cl.set_rc(rc.with_pitch_ref(rc.with_throttle(rc.neutral(), THROTTLE), target_deg * DEG))
        cl.run_seconds(4.0)
        got = cl.plant.theta / DEG
        err = got - target_deg
        check(abs(err) < 2.0, f"ピッチ {target_deg:+.0f}deg に収束",
              f"真値={got:+.3f} 誤差={err:+.3f} deg")
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    cl.run_seconds(3.0)

    # ⚠ ファームはグローバル状態を持つので ClosedLoop は 1 プロセスに 1 つ。
    print("\n--- 4. 突風外乱からの復帰（角速度インパルス）---")
    # 姿勢を直接書き換えると EKF がジャイロバイアスと誤解釈して
    # 周期 20 秒級の緩い振動になる（制御ではなくテストの物理が誤り）。
    mark = len(cl.rows)
    cl.disturb_rates(dp=+120 * DEG, dq=-80 * DEG)
    cl.run_seconds(5.0)
    ph, th = cl.plant.phi / DEG, cl.plant.theta / DEG
    peak = peak_abs(cl, "phi_true", mark) / DEG
    check(abs(ph) < 0.5 and abs(th) < 0.5, "突風(p=+120,q=-80 deg/s) から復帰",
          f"5秒後 phi={ph:+.3f} theta={th:+.3f} deg  (ピーク {peak:.2f} deg)")

    print("\n--- 5. 推定と真値の一致（整定後の区間で評価）---")
    cl.run_seconds(2.0)
    tail = cl.rows[-400:]
    ephi = np.mean([abs(r["phi_est"] - r["phi_true"]) for r in tail]) / DEG
    etheta = np.mean([abs(r["theta_est"] - r["theta_true"]) for r in tail]) / DEG
    check(ephi < 0.5 and etheta < 0.5, "EKF 推定誤差が 0.5deg 未満",
          f"|phi|={ephi:.4f} |theta|={etheta:.4f} deg")

    print("\n--- 6. ロール/ピッチの非対称性（実機ゲインの帰結・参考値）---")
    print("  control.cpp:699-700  phi_pid Kp=1.0  Ti=100000 (実質積分なし) Td=0")
    print("                       theta_pid Kp=1.6 Ti=10000  Td=0.0007")
    print("  -> ロール軸は比例ゲインが低く積分も効かないため、定常偏差が残りやすい。")
    print("     M2 のステップ応答でもロールの誤差がピッチの数倍出ている。")
    print("     実機チューニング時の既知の非対称性として記録する。")

    print("\n" + "=" * 52)
    if failures:
        print(f"FAILED: {len(failures)} 件")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("M2 全項目 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
