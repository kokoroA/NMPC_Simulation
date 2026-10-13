#!/usr/bin/env python3
"""M5: 自動機動（水平旋回・8の字・上昇旋回）の検証。

これが SIL の本命。`20260824_benzen_new_auto_maneuvers.md` と
`20260825_benzen_new_pitch_sign_figure8.md` に「実機軌道は未検証」と
残っていたものを、墜落リスクなしで確かめる。

検証内容:
  1. 水平旋回: 目標値の生成（バンク・機首下げ・ヨー角速度）と 360deg 旋回
  2. 8の字   : 左右一周・交差点でのゼロ通過・正味ヨー変化ゼロ
  3. 上昇旋回: 推力補正の投入と上昇
  4. 追従性能: 指令バンクに実機がどれだけ追従できるか（定量）
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rc
from sim import ClosedLoop

DEG = np.pi / 180.0
THROTTLE = 0.13          # ホバー付近（T_max=2.04 線形仮定）
failures = []


def check(cond, msg, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}{('  ' + extra) if extra else ''}")
    if not cond:
        failures.append(msg)


def run_maneuver(cl, mode, max_s=40.0):
    """モードを開始し、NORMAL に戻るまで回す。区間のログ範囲を返す。"""
    start = len(cl.rows)
    cl.sil.set_flight_mode(mode)
    n = 0
    while n * 0.0025 < max_s:
        cl.run(40)          # 0.1 s
        n += 40
        if cl.sil.flight_mode == 0 and n > 400:
            break
    cl.run_seconds(2.0)     # 収束待ち
    return cl.rows[start:], n * 0.0025


def col(rows, key):
    return np.array([r[key] for r in rows], dtype=float)


def main() -> int:
    print("=== M5: 自動機動の検証 ===\n")
    cl = ClosedLoop()
    cl.arm()
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    cl.run_seconds(2.0)

    # ------------------------------------------------------------ 水平旋回
    print("--- 1. 自動水平旋回 ---")
    rows, dur = run_maneuver(cl, cl.sil.MODE_AUTO_HORIZONTAL_TURN)
    phi_ref, th_ref, rref = col(rows, "phi_ref"), col(rows, "theta_ref"), col(rows, "Rref")
    psi = col(rows, "psi_true")
    hold = np.abs(rref) > 0.9 * np.max(np.abs(rref))    # 定常旋回区間

    check(np.max(phi_ref) / DEG > 17.0, "バンク指令が設定値 18deg に達する",
          f"最大 {np.max(phi_ref)/DEG:.2f} deg")
    check(np.min(th_ref) / DEG < -11.0, "ピッチ指令が機首下げ（負）",
          f"最小 {np.min(th_ref)/DEG:.2f} deg")
    check(np.max(rref) / DEG > 34.0, "ヨー角速度指令が設定値 35deg/s に達する",
          f"最大 {np.max(rref)/DEG:.2f} deg/s")
    total_yaw = (psi[-1] - psi[0]) / DEG
    check(340.0 < total_yaw < 420.0, "一周（360deg 前後）まわる",
          f"{total_yaw:.1f} deg  所要 {dur:.1f} s")
    check(cl.sil.flight_mode == 0, "終了後 NORMAL に戻る")

    # 追従性能（本命の指標）
    phi_true = col(rows, "phi_true")
    phi_true_turn, phi_ref_turn, hold_turn = phi_true, phi_ref, hold
    theta_true_turn = col(rows, "theta_true")
    r_true_turn = col(rows, "r_true")
    q_true_turn = col(rows, "q_true")
    bank_cmd = np.mean(phi_ref[hold]) / DEG
    bank_act = np.mean(phi_true[hold]) / DEG
    ratio_turn = bank_act / bank_cmd
    print(f"      指令バンク {bank_cmd:.2f} deg -> 実バンク {bank_act:.2f} deg  "
          f"(達成率 {100*ratio_turn:.0f} %)")

    # ------------------------------------------------------------ 8の字
    print("\n--- 2. 自動 8 の字 ---")
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    cl.run_seconds(2.0)
    rows, dur = run_maneuver(cl, cl.sil.MODE_AUTO_FIGURE_EIGHT, max_s=45.0)
    phi_ref = col(rows, "phi_ref")
    rref = col(rows, "Rref")
    psi = col(rows, "psi_true")

    check(np.max(phi_ref) / DEG > 15.0 and np.min(phi_ref) / DEG < -15.0,
          "バンク指令が左右両方向に出る",
          f"{np.min(phi_ref)/DEG:+.1f} 〜 {np.max(phi_ref)/DEG:+.1f} deg")
    check(np.max(rref) > 0 and np.min(rref) < 0, "ヨー角速度が正負に反転する",
          f"{np.min(rref)/DEG:+.1f} 〜 {np.max(rref)/DEG:+.1f} deg/s")

    psi_rel = (psi - psi[0]) / DEG
    peak = float(np.max(psi_rel))
    net = float(psi_rel[-1])
    check(320.0 < peak < 400.0, "第1ループで約 360deg まわる", f"最大 {peak:.1f} deg")
    check(abs(net) < 30.0, "第2ループで元の機首方位へ戻る（正味ゼロ）",
          f"正味 {net:+.1f} deg  所要 {dur:.1f} s")

    # 交差点（ヨー角速度が 0 を横切る点）でバンクも 0 か
    sign_change = np.nonzero(np.diff(np.sign(rref)))[0]
    mid = sign_change[len(sign_change) // 2] if len(sign_change) else None
    if mid is not None:
        check(abs(phi_ref[mid]) / DEG < 3.0, "交差点でバンク指令がほぼ 0",
              f"{phi_ref[mid]/DEG:+.2f} deg")
    check(cl.sil.flight_mode == 0, "終了後 NORMAL に戻る")

    # ------------------------------------------------------------ 上昇旋回
    print("\n--- 3. 自動上昇旋回 ---")
    cl.set_rc(rc.with_throttle(rc.neutral(), THROTTLE))
    cl.run_seconds(2.0)
    cl.plant.pos[:] = 0.0
    cl.plant.vel[:] = 0.0
    alt0 = cl.plant.altitude
    rows, dur = run_maneuver(cl, cl.sil.MODE_AUTO_CLIMBING_TURN)
    t_ref = col(rows, "T_ref")
    base = t_ref[0]
    boost = (np.max(t_ref) - base) / base
    check(0.06 < boost < 0.10, "推力補正が設定値 +8% 付近まで入る",
          f"+{100*boost:.2f} %")
    climb = cl.plant.altitude - alt0
    check(climb > 5.0, "旋回中に上昇する", f"{climb:+.2f} m  所要 {dur:.1f} s")
    psi = col(rows, "psi_true")
    check(340.0 < (psi[-1] - psi[0]) / DEG < 420.0, "上昇しながら一周する",
          f"{(psi[-1]-psi[0])/DEG:.1f} deg")
    check(abs(t_ref[-1] - base) / base < 0.02, "終了後に推力補正が戻る",
          f"{t_ref[-1]:.4f} vs {base:.4f}")

    # ------------------------------------------------------------ 追従性能
    print("\n--- 4. バンク角の追従性能（実機で効く指標）---")
    print(f"      水平旋回: 指令 {bank_cmd:.1f} deg -> 実 {bank_act:.1f} deg（達成率 {100*ratio_turn:.0f} %）")
    print()
    print("      原因: 旋回中の運動学的カップリングと、ロール軸の弱いゲイン。")
    print("        phi_dot = p + q sin(phi) tan(theta) + r cos(phi) tan(theta)")
    print("      機首下げ（theta<0）でヨーを回すと r cos(phi) tan(theta) が負に効き、")
    print("      バンクを倒す向きの外乱になる。ロール角度 PID は Kp=1.0 で積分が無い")
    print("      （control.cpp:699 Ti=100000）ため、この定常外乱を打ち消せず偏差が残る。")
    # 釣り合い式を数値で確認する（主張を裏取りする）
    #   定常では phi_dot = 0 なので  p = -(q sin(phi) + r cos(phi)) tan(theta)
    #   ロール角度 PID の出力は Pref = Kp * (phi_ref - phi)   （Kp = 1.0, 積分なし）
    #   したがって  Kp*(phi_ref - phi) ≈ -r cos(phi) tan(theta)  で釣り合う。
    KP_ROLL = 1.0                      # control.cpp:699 phi_pid.set_parameter(1, 100000, 0, ...)
    phi_h = np.mean(phi_true_turn[hold_turn])
    th_h = np.mean(theta_true_turn[hold_turn])
    r_h = np.mean(r_true_turn[hold_turn])
    q_h = np.mean(q_true_turn[hold_turn])
    lhs = KP_ROLL * (np.mean(phi_ref_turn[hold_turn]) - phi_h)
    rhs = -(q_h * np.sin(phi_h) + r_h * np.cos(phi_h)) * np.tan(th_h)
    rel = abs(lhs - rhs) / max(abs(rhs), 1e-9)
    print(f"      釣り合い検算: Kp*(指令-実) = {lhs:+.4f} rad/s")
    print(f"                    -(q sin+r cos) tan(theta) = {rhs:+.4f} rad/s   （相対差 {100*rel:.1f} %）")
    check(rel < 0.15, "釣り合い式が数値的に成立（原因の裏取り）",
          f"相対差 {100*rel:.1f} %")
    check(ratio_turn < 0.75, "【要対処】指令バンクの 75% 未満しか出ていない",
          f"達成率 {100*ratio_turn:.0f} % — 実機では旋回半径が設計より大きくなる")

    print("\n" + "=" * 56)
    if failures:
        print(f"FAILED: {len(failures)} 件")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("M5 全項目 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
