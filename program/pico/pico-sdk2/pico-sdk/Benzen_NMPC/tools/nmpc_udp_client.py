#!/usr/bin/env python3
"""NMPC Wi-Fi UDP client for Benzen_NMPC (Pico 2 W AP mode).

接続手順:
  1. Pico 2 W にファームウェアを書き込み、USB シリアルで
     ``#wifi: AP 'Benzen_NMPC' IP 192.168.4.1`` が出ることを確認
  2. PC の Wi-Fi を Pico の AP に接続
       SSID: Benzen_NMPC
       Pass: benzen2026
  3. このスクリプトを実行

Usage:
  python3 tools/nmpc_udp_client.py --check
  python3 tools/nmpc_udp_client.py
  python3 tools/nmpc_udp_client.py --set-cost Q_phi=200 R_u=20
  python3 tools/nmpc_udp_client.py --show-cost
  python3 tools/nmpc_udp_client.py --show-graph
  python3 tools/nmpc_udp_client.py --log out.csv
"""
from __future__ import annotations

import argparse
import csv
import socket
import struct
import subprocess
import sys
import time

PICO_IP = "192.168.4.1"
CMD_PORT = 5000
TEL_PORT = 5001
AP_SSID = "Benzen_NMPC"
AP_PASS = "benzen2026"

NMPC_UDP_MAGIC = 0xBEEF
# wifi_telemetry.hpp の NMPC_UDP_VER と一致させること
NMPC_UDP_VER = 6
NMPC_PKT_REGISTER = 0x01
NMPC_PKT_REGISTER_ACK = 0x02
NMPC_PKT_SET_COST = 0x10
NMPC_PKT_COST_ACK = 0x11
NMPC_PKT_TEL = 0x20

# control.cpp debug_print_nmpc_bench() の #cols 行と同一
NMPC_BENCH_CSV_COLS = [
    "seq", "arm", "nmpc",
    "phi", "theta", "p", "q", "r",
    "phi_ref", "theta_ref", "psi_ref", "T_ref", "u_base",
    "schedule_alpha", "Q_phi_eff",
    "iter_us", "loop_us", "cost", "reset", "iter_max",
    "reset_cause", "path", "tr1", "cycles",
    "cnt_tr1", "cnt_tr1_ok", "cnt_tr1_skip",
    "cnt_rst_period", "cnt_rst_ext", "cnt_sr2_c", "cnt_sr2_llt", "cnt_rst_after_sr2",
    "u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl",
    "a_hat_p", "a_hat_q", "a_hat_r",
    "d_fr", "d_fl", "d_mr", "d_ml", "d_rr", "d_rl",
    "ftc",
    "failsafe",
    # 受信品質 (ver 5)。rx_flags は SBUS フラグバイト生値 0x04=frame lost / 0x08=failsafe。
    "rx_flags",
    "rx_fl_run",
    "rx_fl_cnt",
]

NMPC_COST_KEYS = [
    "Q_phi", "Q_theta", "Q_p", "Q_q", "Q_r", "R_u", "terminal_scale",
]

SET_COST_DEFAULTS = {
    "Q_phi": 1.0,
    "Q_theta": 1.0,
    "Q_p": 0.1,
    "Q_q": 0.1,
    "Q_r": 0.5,
    "R_u": 20.0,
    "terminal_scale": 10.0,
}

HDR_FMT = "<HBBI"
SET_COST_FMT = HDR_FMT + "7f"
ACK_FMT = HDR_FMT + "7fB"
TEL_FMT = (
    HDR_FMT
    + "BBBBBBBB"  # arm_flag .. failsafe
    + "i"         # cycles_since_reset
    + "8I"        # cnt_tr1_path .. cnt_reset_after_sr2
    + "37f"
    + "BBHI"      # rx_flags, rx_framelost_run, _rsv16, rx_framelost_cnt (ver 5)
)


def pico_subnet_prefix(pico_ip: str) -> str:
    parts = pico_ip.split(".")
    if len(parts) != 4:
        return "192.168.4."
    return f"{parts[0]}.{parts[1]}.{parts[2]}."


def find_local_ip_on_pico_subnet(pico_ip: str) -> str | None:
    """Pico と同じ /24 にいるローカル IP を探す (macOS/Linux ifconfig)。"""
    prefix = pico_subnet_prefix(pico_ip)
    try:
        proc = subprocess.run(
            ["ifconfig"],
            capture_output=True,
            text=True,
            check=False,
            timeout=5,
        )
    except (OSError, subprocess.TimeoutExpired):
        return None

    for line in proc.stdout.splitlines():
        line = line.strip()
        if not line.startswith("inet "):
            continue
        # macOS: inet 192.168.4.2 netmask ...
        # Linux: inet 192.168.4.2  netmask ...
        parts = line.split()
        if len(parts) < 2:
            continue
        ip = parts[1]
        if ip.startswith(prefix) and ip != pico_ip:
            return ip
    return None


def check_network(pico_ip: str) -> tuple[bool, str]:
    local = find_local_ip_on_pico_subnet(pico_ip)
    if local:
        return True, f"OK: この PC は {local} (Pico {pico_ip} へ到達可能なはず)"
    return False, (
        f"NG: {pico_subnet_prefix(pico_ip)}x の IP が見つかりません。\n"
        f"  → Wi-Fi で AP '{AP_SSID}' (pass: {AP_PASS}) に接続してください。\n"
        f"  → Pico の USB シリアルに #wifi: AP ... が出ているか確認してください。"
    )


def print_network_help(pico_ip: str) -> None:
    print(
        "\n[Network is unreachable の対処]\n"
        f"  1. Mac の Wi-Fi を「{AP_SSID}」に切り替える (パスワード: {AP_PASS})\n"
        "  2. 接続後、システム設定で IP が 192.168.4.x になっているか確認\n"
        "  3. Pico を USB 給電し、シリアルログに以下が出るか確認:\n"
        "       #wifi: AP 'Benzen_NMPC' IP 192.168.4.1 ...\n"
        f"  4. 接続確認: python3 {sys.argv[0]} --check\n"
        f"  5. 別ネットワークにいる場合は --pico-ip {pico_ip} を確認\n",
        file=sys.stderr,
    )


def make_hdr(pkt_type: int, seq: int) -> bytes:
    return struct.pack(HDR_FMT, NMPC_UDP_MAGIC, pkt_type, NMPC_UDP_VER, seq)


def send_register(sock: socket.socket, pico: tuple[str, int], seq: int) -> None:
    sock.sendto(make_hdr(NMPC_PKT_REGISTER, seq), pico)


def send_set_cost(sock: socket.socket, pico: tuple[str, int], seq: int, **kwargs) -> dict[str, float]:
    """SET_COST パケット送信。実際に送った全 7 パラメータを返す。"""
    params = dict(SET_COST_DEFAULTS)
    params.update(kwargs)
    pkt = struct.pack(
        SET_COST_FMT,
        NMPC_UDP_MAGIC,
        NMPC_PKT_SET_COST,
        NMPC_UDP_VER,
        seq,
        params["Q_phi"],
        params["Q_theta"],
        params["Q_p"],
        params["Q_q"],
        params["Q_r"],
        params["R_u"],
        params["terminal_scale"],
    )
    sock.sendto(pkt, pico)
    return params


def parse_cost_kv(items: list[str]) -> dict[str, float]:
    out: dict[str, float] = {}
    for item in items:
        if "=" not in item:
            raise ValueError(f"expected KEY=VAL, got {item!r}")
        k, v = item.split("=", 1)
        out[k.strip()] = float(v.strip())
    return out


def parse_tel(data: bytes) -> dict | None:
    if len(data) < struct.calcsize(TEL_FMT):
        return None
    vals = struct.unpack(TEL_FMT, data[: struct.calcsize(TEL_FMT)])
    keys = [
        "magic", "type", "ver", "seq",
        "arm", "nmpc", "reset", "reset_cause", "path", "tr1", "ftc", "failsafe", "cycles",
        "cnt_tr1", "cnt_tr1_ok", "cnt_tr1_skip",
        "cnt_rst_period", "cnt_rst_ext", "cnt_sr2_c", "cnt_sr2_llt", "cnt_rst_after_sr2",
        "phi", "theta", "p", "q", "r",
        "phi_ref", "theta_ref", "psi_ref", "T_ref", "u_base", "schedule_alpha",
        "iter_us", "loop_us", "cost", "iter_max",
        "u_fr", "u_fl", "u_mr", "u_ml", "u_rr", "u_rl",
        "a_hat_p", "a_hat_q", "a_hat_r",
        "d_fr", "d_fl", "d_mr", "d_ml", "d_rr", "d_rl",
        "Q_phi", "Q_theta", "Q_p", "Q_q", "Q_r", "R_u", "terminal_scale",
        "rx_flags", "rx_fl_run", "_rsv16", "rx_fl_cnt",
    ]
    if len(keys) != len(vals):
        return None
    row = dict(zip(keys, vals))
    if row["magic"] != NMPC_UDP_MAGIC or row["type"] != NMPC_PKT_TEL:
        return None
    if row["ver"] != NMPC_UDP_VER:
        return None
    return row


def print_bench_csv_header() -> None:
    """USB シリアル debug_print_nmpc_bench() と同じヘッダー。"""
    print("#nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)")
    print("#reset_cause: 0=none 1=init 2=external 3=period 4=sr2_bad_c 5=sr2_unhealthy")
    print("#path: 0=tr1/sr2 1=reset@start 2=reset@after_sr2")
    print("#cols: " + ",".join(NMPC_BENCH_CSV_COLS))


def format_bench_csv_row(row: dict) -> str:
    """control.cpp の printf 書式と同じ精度で 1 行 CSV を生成。"""
    r = row
    return (
        f"{int(r['seq'])},{int(r['arm'])},{int(r['nmpc'])},"
        f"{r['phi']:.4f},{r['theta']:.4f},{r['p']:.4f},{r['q']:.4f},{r['r']:.4f},"
        f"{r['phi_ref']:.4f},{r['theta_ref']:.4f},{r['psi_ref']:.4f},{r['T_ref']:.4f},{r['u_base']:.4f},"
        f"{r['schedule_alpha']:.4f},{r['Q_phi']:.4f},"
        f"{r['iter_us']:.0f},{r['loop_us']:.0f},{r['cost']:.2f},"
        f"{int(r['reset'])},{r['iter_max']:.0f},"
        f"{int(r['reset_cause'])},{int(r['path'])},{int(r['tr1'])},{int(r['cycles'])},"
        f"{int(r['cnt_tr1'])},{int(r['cnt_tr1_ok'])},{int(r['cnt_tr1_skip'])},"
        f"{int(r['cnt_rst_period'])},{int(r['cnt_rst_ext'])},"
        f"{int(r['cnt_sr2_c'])},{int(r['cnt_sr2_llt'])},{int(r['cnt_rst_after_sr2'])},"
        f"{r['u_fr']:.4f},{r['u_fl']:.4f},{r['u_mr']:.4f},{r['u_ml']:.4f},"
        f"{r['u_rr']:.4f},{r['u_rl']:.4f},"
        f"{r['a_hat_p']:.4f},{r['a_hat_q']:.4f},{r['a_hat_r']:.4f},"
        f"{r['d_fr']:.4f},{r['d_fl']:.4f},{r['d_mr']:.4f},{r['d_ml']:.4f},"
        f"{r['d_rr']:.4f},{r['d_rl']:.4f}"
    )


def bench_csv_dict(row: dict) -> dict:
    """--log 用: シリアル CSV 列のみ抽出。"""
    out = {k: row[k] for k in NMPC_BENCH_CSV_COLS if k in row}
    if "Q_phi_eff" not in out and "Q_phi" in row:
        out["Q_phi_eff"] = row["Q_phi"]
    return out


def cost_active_dict(row: dict) -> dict[str, float]:
    return {k: float(row[k]) for k in NMPC_COST_KEYS}


def format_cost_line(row: dict) -> str:
    """テレメトリ UDP 内の nmpc_cost_get_active() 相当。"""
    r = row
    return (
        f"# cost_active seq={int(r['seq'])} "
        f"Q_phi={r['Q_phi']:.4f} Q_theta={r['Q_theta']:.4f} "
        f"Q_p={r['Q_p']:.4f} Q_q={r['Q_q']:.4f} Q_r={r['Q_r']:.4f} "
        f"R_u={r['R_u']:.4f} terminal_scale={r['terminal_scale']:.4f}"
    )


def format_cost_dict_line(prefix: str, cost: dict[str, float]) -> str:
    return (
        f"{prefix} "
        f"Q_phi={cost['Q_phi']:.4f} Q_theta={cost['Q_theta']:.4f} "
        f"Q_p={cost['Q_p']:.4f} Q_q={cost['Q_q']:.4f} Q_r={cost['Q_r']:.4f} "
        f"R_u={cost['R_u']:.4f} terminal_scale={cost['terminal_scale']:.4f}"
    )


def costs_match(a: dict[str, float], b: dict[str, float], atol: float = 0.05) -> bool:
    return all(abs(a[k] - b[k]) <= atol for k in NMPC_COST_KEYS)


def wait_for_network(pico_ip: str, timeout_s: float) -> bool:
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        ok, _ = check_network(pico_ip)
        if ok:
            return True
        time.sleep(1.0)
    return False


def main() -> int:
    ap = argparse.ArgumentParser(description="Benzen NMPC Wi-Fi UDP client")
    ap.add_argument("--pico-ip", default=PICO_IP)
    ap.add_argument("--set-cost", nargs="*", metavar="KEY=VAL",
                    help="e.g. Q_phi=200 R_u=20 terminal_scale=15")
    ap.add_argument("--log", metavar="CSV", help="save telemetry to CSV")
    ap.add_argument(
        "--show-cost",
        action="store_true",
        help="テレメトリの active コスト重みを表示 (変化時)。--set-cost 時は自動 ON",
    )
    ap.add_argument(
        "--show-graph",
        action="store_true",
        help="リアルタイムグラフ表示 (姿勢・モータ・ソルバ)",
    )
    ap.add_argument(
        "--graph-window",
        type=float,
        default=60.0,
        metavar="SEC",
        help="グラフの表示時間幅 [s] (--show-graph 時, default: 60)",
    )
    ap.add_argument(
        "--print-csv",
        action="store_true",
        help="--show-graph 時も CSV を標準出力に出す",
    )
    ap.add_argument("--duration", type=float, default=0.0,
                    help="stop after N seconds (0=forever)")
    ap.add_argument("--check", action="store_true",
                    help="ネットワーク接続だけ確認して終了")
    ap.add_argument("--wait", type=float, default=0.0, metavar="SEC",
                    help="AP 接続を最大 SEC 秒待ってから開始")
    args = ap.parse_args()

    if args.check:
        ok, msg = check_network(args.pico_ip)
        print(msg)
        return 0 if ok else 1

    if args.wait > 0:
        print(f"AP 接続を最大 {args.wait:.0f} 秒待機...")
        if not wait_for_network(args.pico_ip, args.wait):
            ok, msg = check_network(args.pico_ip)
            print(msg, file=sys.stderr)
            print_network_help(args.pico_ip)
            return 1
    else:
        ok, msg = check_network(args.pico_ip)
        if not ok:
            print(msg, file=sys.stderr)
            print_network_help(args.pico_ip)
            return 1
        print(msg)

    pico_cmd = (args.pico_ip, CMD_PORT)

    cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tel.bind(("0.0.0.0", TEL_PORT))
    tel.settimeout(1.0)
    cmd.settimeout(2.0)

    seq = 0
    try:
        send_register(cmd, pico_cmd, seq)
    except OSError as e:
        if e.errno in (51, 101, 113):  # unreachable / network down / no route
            print(f"送信失敗: {e}", file=sys.stderr)
            print_network_help(args.pico_ip)
            return 1
        raise

    seq += 1
    try:
        ack_data, ack_from = cmd.recvfrom(256)
        if len(ack_data) >= struct.calcsize(HDR_FMT):
            magic, ptype, ver, aseq = struct.unpack(
                HDR_FMT, ack_data[: struct.calcsize(HDR_FMT)]
            )
            if magic == NMPC_UDP_MAGIC and ptype == NMPC_PKT_REGISTER_ACK:
                if ver != NMPC_UDP_VER:
                    print(
                        f"警告: REGISTER_ACK の ver={ver} がクライアント期待 ver={NMPC_UDP_VER} と不一致",
                        file=sys.stderr,
                    )
                print(f"REGISTER_ACK from {ack_from[0]}:{ack_from[1]} seq={aseq} ver={ver}")
            else:
                print(f"REGISTER 送信済み (応答 type=0x{ptype:02x} ver={ver})", file=sys.stderr)
    except socket.timeout:
        print(
            "警告: REGISTER_ACK なし — Pico がパケットを受信していない可能性があります",
            file=sys.stderr,
        )

    print(f"テレメトリ待受 UDP {TEL_PORT}")
    print_csv = args.print_csv or not args.show_graph
    if print_csv:
        print_bench_csv_header()

    graph = None
    if args.show_graph:
        try:
            from nmpc_udp_graph import RealtimeGraph
        except ImportError as e:
            print(f"matplotlib が必要です: pip install matplotlib ({e})", file=sys.stderr)
            return 1
        graph = RealtimeGraph(window_s=args.graph_window)
        print(f"リアルタイムグラフ表示 (window={args.graph_window:.0f}s)。ウィンドウを閉じると終了。")

    show_cost = args.show_cost or bool(args.set_cost)
    expected_cost: dict[str, float] | None = None
    cost_applied_announced = False
    last_cost_tuple: tuple[float, ...] | None = None

    if args.set_cost:
        cost_kv = parse_cost_kv(args.set_cost)
        try:
            expected_cost = send_set_cost(cmd, pico_cmd, seq, **cost_kv)
        except OSError as e:
            print(f"SET_COST 送信失敗: {e}", file=sys.stderr)
            print_network_help(args.pico_ip)
            return 1
        print(f"SET_COST 送信: {cost_kv}")
        print(format_cost_dict_line("# cost EXPECT (core1 apply 後にテレメで確認):", expected_cost))
        seq += 1
        try:
            ack_data, _ = cmd.recvfrom(256)
            if len(ack_data) >= struct.calcsize(ACK_FMT):
                ack = struct.unpack(ACK_FMT, ack_data[: struct.calcsize(ACK_FMT)])
                ack_dict = dict(
                    zip(
                        [
                            "magic", "type", "ver", "seq",
                            "Q_phi", "Q_theta", "Q_p", "Q_q", "Q_r",
                            "R_u", "terminal_scale", "applied",
                        ],
                        ack,
                    )
                )
                print("ACK (受付確認):", ack_dict)
                print(
                    "  ※ ACK の Q/R は apply 前の active 値です。"
                    "反映確認はテレメの # cost_active を参照してください。",
                    file=sys.stderr,
                )
        except socket.timeout:
            print("警告: ACK タイムアウト (Pico が応答しない)", file=sys.stderr)
    elif show_cost:
        print("# cost_active: テレメトリの active コスト重み (変化時に表示)")

    writer = None
    csv_file = None
    if args.log:
        csv_file = open(args.log, "w", newline="")
        writer = csv.DictWriter(csv_file, fieldnames=NMPC_BENCH_CSV_COLS)
        csv_file.write(
            "#nmpc_bench CSV (10Hz) iter_us_limit=10000 (100Hz)\n"
            "#reset_cause: 0=none 1=init 2=external 3=period 4=sr2_bad_c 5=sr2_unhealthy\n"
            "#path: 0=tr1/sr2 1=reset@start 2=reset@after_sr2\n"
            f"#cols: {','.join(NMPC_BENCH_CSV_COLS)}\n"
        )
        writer.writeheader()
        print(f"logging to {args.log}")

    t0 = time.time()
    no_data_warned = False
    try:
        while True:
            if graph is not None and not graph.poll():
                print("\nグラフウィンドウを閉じました")
                break
            if args.duration > 0 and (time.time() - t0) >= args.duration:
                break
            try:
                data, addr = tel.recvfrom(2048)
            except socket.timeout:
                if graph is not None:
                    graph.poll()
                if not no_data_warned and (time.time() - t0) > 5.0:
                    print(
                        "5秒間テレメトリなし。REGISTER 後に Pico が動作中か確認してください。",
                        file=sys.stderr,
                    )
                    no_data_warned = True
                continue
            row = parse_tel(data)
            if not row:
                if len(data) >= struct.calcsize(HDR_FMT):
                    _, ptype, ver, _ = struct.unpack(
                        HDR_FMT, data[: struct.calcsize(HDR_FMT)]
                    )
                    if ptype == NMPC_PKT_TEL and ver != NMPC_UDP_VER:
                        print(
                            f"警告: テレメ ver={ver} が不一致 (期待 ver={NMPC_UDP_VER})。"
                            "ファームウェアとクライアントの NMPC_UDP_VER を揃えてください。",
                            file=sys.stderr,
                        )
                    elif len(data) != struct.calcsize(TEL_FMT):
                        print(
                            f"警告: テレメサイズ {len(data)}B != 期待 "
                            f"{struct.calcsize(TEL_FMT)}B (ver={NMPC_UDP_VER})。"
                            "ファームウェアを再ビルドしたか確認してください。",
                            file=sys.stderr,
                        )
                continue
            no_data_warned = False
            if print_csv:
                print(format_bench_csv_row(row))
            if writer:
                writer.writerow(bench_csv_dict(row))
            if graph is not None:
                graph.append(row)

            if show_cost:
                active = cost_active_dict(row)
                cost_tuple = tuple(active[k] for k in NMPC_COST_KEYS)
                if last_cost_tuple is None or cost_tuple != last_cost_tuple:
                    print(format_cost_line(row))
                    last_cost_tuple = cost_tuple
                    if expected_cost is not None and not cost_applied_announced:
                        if costs_match(active, expected_cost):
                            print("# cost APPLIED: テレメトリが SET_COST と一致しました")
                            cost_applied_announced = True
                            expected_cost = None
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        if csv_file:
            csv_file.close()
        if graph is not None:
            graph.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
