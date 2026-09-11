# Benzen_new Wi-Fi リアルタイムテレメ（PID）

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_new/`（wifi_telemetry, control, pico_copter, tools/pid_udp_client.py）
- **タグ**: #Benzen_new #Pico #pico2_w #WiFi #テレメ #UDP #CYW43

## 背景 / 症状

Benzen_new は Pico 2 W 向け移植済みだが、CYW43 は LED 専用（`pico_cyw43_arch_none`）でリアルタイムテレメは無かった。
Benzen_NMPC では AP + UDP（CMD 5000 / TEL 5001）で 10 Hz テレメが実証済み。

## 原因 / 判断根拠

- NMPC の `NmpcTelPkt` はソルバ診断が中心で PID 制御には過剰。ペイロードは PID 向けに新規定義（ver=1）。
- SSID は `Benzen_new`（NMPC の `Benzen_NMPC` と同時運用時の衝突回避）。
- 400 Hz / 100 Hz ループから Wi-Fi を触らない。NMPC と同じく core1@100Hz スナップショット + core0 main@10Hz 送信。
- `SET_COST` 相当（PID ゲイン書き換え）は v1 範囲外。

## 対応

- `Benzen_NMPC` から `wifi_telemetry`, `dhcpserver`, `lwipopts.h` を移植し PID 用に改変。
- `CMakeLists.txt`: `pico_cyw43_arch_lwip_threadsafe_background`, `PID_WIFI_TELEM=1`, AP `192.168.4.1`。
- `board_led`: `cyw43_arch_init()` は `wifi_telemetry_init()` のみ（二重 init 回避）。
- `control.cpp`: `g_pid_telem` を 100 Hz 更新、`debug_print_pid_telem()` で 10 Hz UDP 送信。duty は global へ書き戻し。
- ホスト: `tools/pid_udp_client.py`（REGISTER + CSV 10 Hz）。

## 検証結果

- `build_fresh` で `pico_copter` ビルド成功。UF2 family `0xe48bff57`（RP2350-ARM-S）。
- パケットサイズ 112 B（Python `TEL_FMT` と一致）。
- 実機 AP 接続・UDP 受信は未検証。

## 次に触る人へ

- 書き込み後 USB で `#wifi: AP 'Benzen_new' ...` を確認してから PC を AP に接続。
- `python3 tools/pid_udp_client.py --check` → 本番受信。
- RC 2.4 GHz と AP が同帯域。飛行試験では干渉を一度確認すること。
