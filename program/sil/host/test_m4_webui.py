#!/usr/bin/env python3
"""M4: WebUI の検証。

サーバを別プロセスで起動して HTTP 経由で叩く。
（ファームはグローバル状態を持ち 1 プロセス 1 インスタンスなので、
  テストプロセス内に直接 Simulation を作ることはできない。）

検証内容:
  1. 起動時にウォームアップが早送りされ Arm_flag=3 になる
  2. 静的ファイルが配信される
  3. POST /input でアーム・操縦できる
  4. SSE /stream がテレメトリを流す
  5. 停止判定の修正が効いている（common_duty 基準）
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
PORT = 8477
BASE = f"http://127.0.0.1:{PORT}"
failures = []


def check(cond, msg, extra=""):
    print(f"  [{'PASS' if cond else 'FAIL'}] {msg}{('  ' + extra) if extra else ''}")
    if not cond:
        failures.append(msg)


def get(path, timeout=5):
    with urllib.request.urlopen(BASE + path, timeout=timeout) as r:
        return r.status, r.read()


def post(payload, timeout=5):
    req = urllib.request.Request(
        BASE + "/input", data=json.dumps(payload).encode(),
        headers={"Content-Type": "application/json"}, method="POST")
    with urllib.request.urlopen(req, timeout=timeout) as r:
        return r.status


def state():
    return json.loads(get("/state")[1])


def wait_until(pred, timeout=20.0, poll=0.2):
    end = time.time() + timeout
    while time.time() < end:
        try:
            if pred(state()):
                return True
        except Exception:
            pass
        time.sleep(poll)
    return False


def main() -> int:
    print("=== M4: WebUI の検証 ===\n")
    proc = subprocess.Popen(
        [sys.executable, str(HERE / "webserver.py"), "--port", str(PORT), "--speed", "4"],
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    try:
        print("--- 1. 起動とウォームアップ ---")
        ready = False
        for _ in range(120):
            try:
                get("/state", timeout=1)
                ready = True
                break
            except (urllib.error.URLError, ConnectionError, TimeoutError):
                time.sleep(0.5)
        check(ready, "サーバが応答する")
        if not ready:
            return 1
        st = state()
        check(st["arm"] == 3, "ウォームアップ済みで Arm_flag=3", f"arm={st['arm']}")
        check(st["t"] > 14.0, "ウォームアップ分の時間が進んでいる", f"t={st['t']:.1f}s")

        print("\n--- 2. 静的ファイル ---")
        for path, needle in (("/", b"Benzen_new SIL"),
                             ("/app.js", b"EventSource"),
                             ("/viz3d.js", b"class Viz3D")):
            code, body = get(path)
            check(code == 200 and needle in body, f"{path} が配信される",
                  f"{len(body)} B")
        code, _ = get("/state")
        check(code == 200, "/state が JSON を返す")

        print("\n--- 3. アームと操縦 ---")
        post({"arm": True})
        check(wait_until(lambda s: s["arm"] == 2, 15), "POST でアームできる",
              f"arm={state()['arm']}")

        # スロットルを上げる。姿勢制御が生きるのは common_duty >= Flight_duty(0.1)
        post({"throttle": 0.30, "roll": 0.0, "pitch": 0.0})
        check(wait_until(lambda s: s["common_duty"] > 0.6, 10), "スロットル入力が反映される",
              f"common_duty={state()['common_duty']:.3f}")
        st = state()
        check(all(d > 0.5 for d in st["duty"]), "6 モータが回っている",
              str([round(d, 3) for d in st["duty"]]))

        post({"throttle": 0.30, "roll": 0.5})
        ok = wait_until(lambda s: s["phi_ref"] > 0.05, 10)
        check(ok, "エルロン入力が目標ロールになる",
              f"phi_ref={state()['phi_ref']*57.3:.2f} deg")
        ok = wait_until(lambda s: abs(s["phi"] - s["phi_ref"]) < 0.05, 15)
        check(ok, "機体が目標ロールに追従する",
              f"phi={state()['phi']*57.3:.2f} / ref={state()['phi_ref']*57.3:.2f} deg")

        print("\n--- 4. SSE ストリーム ---")
        got = []
        with urllib.request.urlopen(BASE + "/stream", timeout=5) as r:
            deadline = time.time() + 1.0
            while time.time() < deadline and len(got) < 20:
                line = r.readline()
                if line.startswith(b"data: "):
                    got.append(json.loads(line[6:]))
        check(len(got) >= 10, "SSE がテレメトリを流す", f"{len(got)} 件 / 1 秒")
        check(all("phi" in g and "duty" in g for g in got), "各フレームに必要な項目がある")
        check(got[-1]["t"] > got[0]["t"], "時刻が進んでいる")

        print("\n--- 5. 停止判定の修正確認 ---")
        # 修正前は common_duty 0.444 未満で停止していた。修正後は 0.1 未満で停止。
        post({"throttle": 0.08})     # common_duty ≈ 0.178 -> 回るはず
        time.sleep(1.0)
        st = state()
        check(st["common_duty"] > 0.1 and max(st["duty"]) > 0.1,
              "common_duty 0.1〜0.444 でもモータが回る（修正の効果）",
              f"common_duty={st['common_duty']:.3f} duty={round(max(st['duty']),3)}")
        post({"throttle": 0.02})     # common_duty ≈ 0.044 -> 停止するはず
        time.sleep(1.0)
        st = state()
        check(max(st["duty"]) <= 0.02,
              "common_duty < 0.1 で停止する",
              f"common_duty={st['common_duty']:.3f} duty={round(max(st['duty']),3)}")

    finally:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    print("\n" + "=" * 52)
    if failures:
        print(f"FAILED: {len(failures)} 件")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("M4 全項目 PASS")
    return 0


if __name__ == "__main__":
    sys.exit(main())
