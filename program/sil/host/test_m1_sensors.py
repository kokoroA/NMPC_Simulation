#!/usr/bin/env python3
"""M1: センサ注入 -> EKF が正しい姿勢を出すことの検証。

SIL で最も壊れやすいのが単位と符号。ここを飛ばすと EKF が静かに破綻し、
「なぜか飛ばない」という形でしか症状が出なくなる。

検証内容:
  1. 水平静止を注入 -> Phi, Theta が 0 に収束する
  2. 既知の傾きを注入 -> EKF がその傾きに収束する（符号込み）
  3. ジャイロ符号 -> 角速度 Wp/Wq/Wr が注入値と一致する
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import rc
from sensors import SensorModel, euler_rates_to_body_rates, smoothstep
from sil_bridge import Sil

DEG = np.pi / 180.0
failures = []


def check(cond, msg, extra=""):
    mark = "PASS" if cond else "FAIL"
    print(f"  [{mark}] {msg}{('  ' + extra) if extra else ''}")
    if not cond:
        failures.append(msg)


DT = 0.0025  # 400Hz


def run_level(sil, sm, n, phi=0.0, theta=0.0, psi=0.0, p=0.0, q=0.0, r=0.0):
    """指定姿勢を n tick 注入する（静止）。"""
    acc, gyr, mag = sm.raw_from_state(phi, theta, psi, p, q, r)
    for _ in range(n):
        sil.set_imu(acc, gyr, mag)
        sil.tick()


def slew(sil, sm, phi0, theta0, phi1, theta1, duration_s, psi=0.0):
    """姿勢を滑らかに動かす。**角速度も整合させて注入する。**

    角速度 0 のまま姿勢だけ動かすと EKF はそれをジャイロバイアスと解釈し、
    収束が極端に遅くなる。実機で起きることを再現するには両方必要。
    """
    n = int(duration_s / DT)
    for i in range(n):
        u0, u1 = i / n, (i + 1) / n
        s0, s1 = smoothstep(u0), smoothstep(u1)
        phi_a, theta_a = phi0 + (phi1 - phi0) * s0, theta0 + (theta1 - theta0) * s0
        phi_b, theta_b = phi0 + (phi1 - phi0) * s1, theta0 + (theta1 - theta0) * s1
        phi_dot, theta_dot = (phi_b - phi_a) / DT, (theta_b - theta_a) / DT
        p, q, r = euler_rates_to_body_rates(phi_a, theta_a, phi_dot, theta_dot, 0.0)
        acc, gyr, mag = sm.raw_from_state(phi_a, theta_a, psi, p, q, r)
        sil.set_imu(acc, gyr, mag)
        sil.tick()


def main() -> int:
    sm = SensorModel()
    sil = Sil()
    sil.set_rc(rc.neutral())

    print("=== M1: センサ注入と EKF の検証 ===\n")
    print("--- 1. バイアス収集 + EKF ウォームアップ（水平静止）---")
    # AVERAGE=2000 でジャイロ/地磁気バイアス、KALMANWAIT=6000 まで EKF が回る
    run_level(sil, sm, 1999)
    st = sil.state()
    print(f"  tick 1999: arm={sil.arm_flag}  (バイアス収集中、EKF はまだ動かない)")

    run_level(sil, sm, 3999)   # -> BiasCounter 5998
    st = sil.state()
    print(f"  tick 5998: arm={sil.arm_flag}  phi={st['phi']/DEG:+.3f}deg  theta={st['theta']/DEG:+.3f}deg")
    check(abs(st["phi"]) < 1.0 * DEG, "水平静止で Phi が 1deg 未満", f"phi={st['phi']/DEG:+.4f}deg")
    check(abs(st["theta"]) < 1.0 * DEG, "水平静止で Theta が 1deg 未満", f"theta={st['theta']/DEG:+.4f}deg")

    # ここを越えると Arm_flag=3 になり sensor_read/EKF が止まる
    run_level(sil, sm, 5)
    print(f"  tick 6003: arm={sil.arm_flag}  (3 = ロック待ち)")
    check(sil.arm_flag == 3, "ウォームアップ完了で Arm_flag=3")

    print("\n--- 2. アーム操作（lock_com ジェスチャ）---")
    acc, gyr, mag = sm.raw_from_state(0, 0, 0)
    for _ in range(80):                       # 50 回超保持で LockMode=1
        sil.set_imu(acc, gyr, mag)
        sil.set_rc(rc.arm_gesture())
        sil.tick()
    sil.set_rc(rc.neutral())                  # 解除で Arm_flag=2
    for _ in range(5):
        sil.set_imu(acc, gyr, mag)
        sil.tick()
    print(f"  arm={sil.arm_flag}  (2 = 飛行可能)")
    check(sil.arm_flag == 2, "ジェスチャで Arm_flag=2 へ遷移")

    # ⚠ スロットルが Flight_duty(0.05) 未満の間、実機は毎周期
    #    Phi_bias = Phi として姿勢ゼロ点を取り直す（地上での姿勢トリム）。
    #    姿勢を「生かす」にはスロットルを上げる必要がある。
    print("\n--- 3. スロットル上げ（姿勢ゼロ点取得を止める）---")
    sil.set_rc(rc.with_throttle(rc.neutral(), 0.30))
    run_level(sil, sm, 40)
    d = sil.debug()
    thr_ok = d["T_ref"] / 7.4 >= 0.05
    check(thr_ok, "T_ref が Flight_duty を超えた", f"T_ref={d['T_ref']:.3f} duty比={d['T_ref']/7.4:.3f}")

    print("\n--- 4. 既知の傾きに EKF が追従するか ---")
    cur_phi, cur_theta = 0.0, 0.0
    for name, phi_t, theta_t in [
        ("ロール +10deg", +10 * DEG, 0.0),
        ("ロール -10deg", -10 * DEG, 0.0),
        ("ピッチ +10deg", 0.0, +10 * DEG),
        ("ピッチ -10deg", 0.0, -10 * DEG),
        ("複合 (+8,-6)", +8 * DEG, -6 * DEG),
    ]:
        slew(sil, sm, cur_phi, cur_theta, phi_t, theta_t, 1.5)   # 1.5s で回す
        run_level(sil, sm, 1200, phi=phi_t, theta=theta_t)       # 3s 保持
        cur_phi, cur_theta = phi_t, theta_t
        st = sil.state()
        ephi = (st["phi"] - phi_t) / DEG
        etheta = (st["theta"] - theta_t) / DEG
        ok = abs(ephi) < 1.5 and abs(etheta) < 1.5
        check(ok, f"{name} に追従",
              f"phi={st['phi']/DEG:+.2f}(誤差{ephi:+.2f})  theta={st['theta']/DEG:+.2f}(誤差{etheta:+.2f})")

    print("\n--- 5. ジャイロの符号と大きさ ---")
    for name, p_t, q_t, r_t in [
        ("ロールレート +30deg/s", +30 * DEG, 0.0, 0.0),
        ("ピッチレート +30deg/s", 0.0, +30 * DEG, 0.0),
        ("ヨーレート  +30deg/s", 0.0, 0.0, +30 * DEG),
    ]:
        acc, gyr, mag = sm.raw_from_state(0, 0, 0, p_t, q_t, r_t)
        sil.set_imu(acc, gyr, mag)
        for _ in range(8):
            sil.tick()
        st = sil.state()
        # state の p,q,r はバイアス補正後。バイアスは水平静止で取ったので ~0。
        got = np.array([st["p"], st["q"], st["r"]]) / DEG
        want = np.array([p_t, q_t, r_t]) / DEG
        ok = np.allclose(got, want, atol=1.0)
        check(ok, f"{name} が一致", f"got={np.round(got,2)} want={np.round(want,2)}")
        # 姿勢を汚さないよう静止に戻す
        run_level(sil, sm, 200)

    print("\n--- 6. ヨー角推定について（既知の制約）---")
    st = sil.state()
    print(f"  psi = {st['psi']/DEG:+.3f} deg （常に 0）")
    print("  control.cpp:1256 で Psi = Xn_est_3（ライントレース用カルマンの角度ズレ）に")
    print("  上書きされる。地磁気由来の Psi = atan2(e12,e11) は破棄されている。")
    print("  Kalman_holizontal は Flight_mode==LINETRACE かつ i2c_connect==1 でしか呼ばれず、")
    print("  SIL にはカメラ/I2C が無いので 0 のままが正しい。")
    print("  通常飛行では control.cpp:1350 の Rref = Psi_ref により")
    print("  Psi_ref はヨー角速度指令として直接使われるため、ヨー角推定は不要。")
    print("  -> 自動旋回・8の字はヨー角速度で動くので M5 に影響しない。")

    print("\n" + "=" * 52)
    if failures:
        print(f"FAILED: {len(failures)} 件")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("M1 全項目 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
