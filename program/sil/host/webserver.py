#!/usr/bin/env python3
"""M4: SIL の WebUI サーバ。

**依存ゼロ**で作る。この venv には FastAPI も websockets も無く、
可視化のためだけに依存を増やしたくない。標準ライブラリの http.server と
Server-Sent Events（ブラウザ標準・ライブラリ不要）で足りる。

    python3 webserver.py [--port 8420] [--speed 1.0]

起動すると:
  1. ウォームアップ(6000 tick)を最高速で早送りして Arm_flag=3 まで進める
  2. 以降は実時間ペースで回し、ブラウザから操縦できる
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import firmware_consts as fc
import rc
from sim import DT, ClosedLoop

WEB_DIR = Path(__file__).resolve().parent.parent / "web"
DEG = 180.0 / np.pi


class Simulation:
    """バックグラウンドで閉ループを回し、状態のスナップショットを配る。"""

    def __init__(self, speed: float = 1.0):
        self.cl = ClosedLoop()
        self.lock = threading.Lock()
        self.speed = speed
        self.running = True
        self.paused = False

        # 操縦入力（0..1 / -1..1 の正規化値）
        self.inp = {"throttle": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0}
        self.arm_hold = 0          # lock_com ジェスチャの保持 tick 数
        self.request_arm = False
        self.request_reset = False
        self.status = "ウォームアップ中"

        self._snap: dict = {}
        self._thread = threading.Thread(target=self._loop, daemon=True)

    # --- 起動 -------------------------------------------------------------
    def start(self):
        # ウォームアップは実時間で 15 秒かかるので最高速で飛ばす
        self.cl.set_rc(rc.neutral())
        self.cl.run(6010)
        self.status = ("アーム待ち（Space 長押し）" if self.cl.sil.arm_flag == 3
                       else f"想定外の状態 arm={self.cl.sil.arm_flag}")
        self._thread.start()

    def stop(self):
        self.running = False

    # --- 入力 -------------------------------------------------------------
    def set_input(self, data: dict):
        with self.lock:
            for k in ("throttle", "roll", "pitch", "yaw"):
                if k in data:
                    v = float(data[k])
                    self.inp[k] = max(0.0, min(1.0, v)) if k == "throttle" else max(-1.0, min(1.0, v))
            if data.get("arm"):
                self.request_arm = True
            if data.get("reset"):
                self.request_reset = True
            if "paused" in data:
                self.paused = bool(data["paused"])
            if "speed" in data:
                self.speed = max(0.1, min(4.0, float(data["speed"])))

    # --- チャンネル生成 ---------------------------------------------------
    def _channels(self) -> list[int]:
        ch = rc.neutral()
        i = self.inp
        ch = rc.with_throttle(ch, i["throttle"])
        ch = rc.with_roll_ref(ch, i["roll"] * rc.ROLL_FULL_SCALE_RAD * 0.12)
        ch = rc.with_pitch_ref(ch, i["pitch"] * rc.PITCH_FULL_SCALE_RAD * 0.25)
        # ラダーはヨー角速度指令（Psi_ref -> Rref）。中立からの偏差で与える。
        mid = (rc.CH_MAX[0] + rc.CH_MIN[0]) * 0.5
        half = (rc.CH_MAX[0] - rc.CH_MIN[0]) * 0.5
        ch[rc.RUDDER] = int(round(mid + i["yaw"] * half))
        return ch

    # --- メインループ -----------------------------------------------------
    def _loop(self):
        next_wall = time.perf_counter()
        while self.running:
            with self.lock:
                if self.request_reset:
                    self.cl.plant.reset()
                    self.request_reset = False
                if self.request_arm and self.cl.sil.arm_flag == 3:
                    self.arm_hold = 80          # lock_com は 50 回超の保持が要る
                    self.request_arm = False

                if self.arm_hold > 0:
                    self.cl.set_rc(rc.arm_gesture())
                    self.arm_hold -= 1
                    self.status = "アーム操作中..."
                elif not self.paused:
                    self.cl.set_rc(self._channels())

                if not self.paused:
                    self.cl.step()
                    if self.arm_hold == 0 and self.cl.sil.arm_flag == 2:
                        self.status = "飛行中"
                    elif self.cl.sil.arm_flag == 3 and self.arm_hold == 0:
                        self.status = "アーム待ち（Space 長押し）"

            # 実時間ペーシング
            next_wall += DT / max(self.speed, 1e-6)
            sleep = next_wall - time.perf_counter()
            if sleep > 0:
                time.sleep(sleep)
            elif sleep < -0.25:      # 大きく遅れたら諦めて追いつく
                next_wall = time.perf_counter()

    # --- 出力 -------------------------------------------------------------
    def snapshot(self) -> dict:
        with self.lock:
            cl = self.cl
            pl = cl.plant
            st = cl.sil.state()
            duty = cl.sil.duty()
            return {
                "t": round(cl.t, 3),
                "status": self.status,
                "arm": cl.sil.arm_flag,
                "mode": cl.sil.flight_mode,
                "paused": self.paused,
                "speed": self.speed,
                # 真値（可視化用）
                "phi": pl.phi, "theta": pl.theta, "psi": pl.psi,
                "p": float(pl.rates[0]), "q": float(pl.rates[1]), "r": float(pl.rates[2]),
                "alt": pl.altitude,
                "pos": [float(pl.pos[0]), float(pl.pos[1])],
                # ファーム内部
                "phi_est": st["phi"], "theta_est": st["theta"],
                "phi_ref": st["phi_ref"], "theta_ref": st["theta_ref"],
                "Rref": st["Rref"], "T_ref": st["T_ref"],
                "duty": [round(d, 4) for d in duty],
                "common_duty": st["T_ref"] * fc.T_REF_COE,
                "input": dict(self.inp),
            }


SIM: Simulation | None = None


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, *a):        # アクセスログは出さない
        pass

    def _send(self, code, ctype, body: bytes, extra=None):
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        for k, v in (extra or {}).items():
            self.send_header(k, v)
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        path = self.path.split("?")[0]
        if path == "/":
            path = "/index.html"

        if path == "/stream":
            return self._sse()

        if path == "/state":
            return self._send(200, "application/json",
                              json.dumps(SIM.snapshot()).encode())

        f = (WEB_DIR / path.lstrip("/")).resolve()
        if not str(f).startswith(str(WEB_DIR.resolve())) or not f.is_file():
            return self._send(404, "text/plain; charset=utf-8", "見つかりません".encode())
        ctype = {".html": "text/html; charset=utf-8",
                 ".js": "application/javascript; charset=utf-8",
                 ".css": "text/css; charset=utf-8"}.get(f.suffix, "application/octet-stream")
        self._send(200, ctype, f.read_bytes())

    def _sse(self):
        """Server-Sent Events でテレメトリを流す。WebSocket ライブラリ不要。"""
        self.send_response(200)
        self.send_header("Content-Type", "text/event-stream; charset=utf-8")
        self.send_header("Cache-Control", "no-cache")
        self.send_header("Connection", "keep-alive")
        self.end_headers()
        try:
            while SIM.running:
                payload = json.dumps(SIM.snapshot())
                self.wfile.write(f"data: {payload}\n\n".encode())
                self.wfile.flush()
                time.sleep(0.02)          # 50 Hz
        except (BrokenPipeError, ConnectionResetError):
            pass

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        try:
            data = json.loads(self.rfile.read(length) or b"{}")
        except json.JSONDecodeError:
            return self._send(400, "application/json", b'{"error":"bad json"}')
        SIM.set_input(data)
        self._send(200, "application/json", b'{"ok":true}')


def main() -> int:
    global SIM
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8420)
    ap.add_argument("--speed", type=float, default=1.0, help="実時間倍率")
    args = ap.parse_args()

    print("=== SIL WebUI ===")
    print("  ウォームアップを早送り中...")
    SIM = Simulation(speed=args.speed)
    SIM.start()
    print(f"  {SIM.status}")
    print(f"\n  ブラウザで開く:  http://127.0.0.1:{args.port}/")
    print("  Ctrl-C で終了\n")

    srv = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        print("\n終了します")
    finally:
        SIM.stop()
        srv.server_close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
