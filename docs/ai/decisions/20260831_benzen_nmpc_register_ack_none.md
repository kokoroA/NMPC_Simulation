# REGISTER_ACK なしは「Wi-Fi 接続成功」ではなく REGISTER 未完了

- **日付**: 2026-08-31
- **種別**: investigation
- **対象**: `Benzen_NMPC/wifi_telemetry.cpp`, `tools/nmpc_udp_client.py`
- **タグ**: #Benzen_NMPC #Pico #pico2_w #WiFi #テレメ #UDP #REGISTER_ACK #NMPC_UDP_VER #5000 #5001

## 背景 / 症状

PC が Pico AP に繋がり、安全解除してモータも回った状態で
`python3 tools/nmpc_udp_client.py --log 0831.csv --show-graph` を実行。

```
OK: この PC は 192.168.4.2 (Pico 192.168.4.1 へ到達可能なはず)
警告: REGISTER_ACK なし — Pico がパケットを受信していない可能性があります
テレメトリ待受 UDP 5001
5秒間テレメトリなし。REGISTER 後に Pico が動作中か確認してください。
```

2 回連続で同じ。飛行制御は生きている。

## 原因 / 判断根拠

**確認済み事実。**

- テレメはブロードキャストしない。`wifi_telemetry_send_bench()` は
  `s_client_known` が真のときだけ 192.168.4.x:5001 へユニキャストする。
  `s_client_known` は REGISTER / SET_COST 受信時にしか立たない。
- クライアントは REGISTER を **1 回だけ** 送り、ACK を 2 秒待ち、以降は 5001 を待つだけ。
  再送しない。ACK が無い時点でテレメは原理的に来ない。
- `--check` / 「OK: 192.168.4.2」は `ifconfig` に 192.168.4.x があることだけを見る。
  ping も UDP も送らない。「到達可能なはず」は未検証。
- 現行ソースはクライアントもファームも `NMPC_UDP_VER=3`。
  `cmd_recv` は `magic` か `ver` が違うと **ログ無しで破棄**する。
  マニュアル `nmpc_udp_client.md` はまだ ver=2 のまま（ソースと食い違い）。
- `Benzen_new` も AP `192.168.4.1`、magic は同じ `0xBEEF`、ただし `PID_UDP_VER=1`。
  そちらに繋がると NMPC クライアントの ver=3 REGISTER は黙って捨てられる。
- 現行 `build_fresh` は `NMPC_WIFI_TELEM=1`。`board_led` の二重 `cyw43_arch_init` は既に回避済み。
- SDK は lwIP コールバック内の `cyw43_arch_lwip_begin` を harmless と明記。再入デッドロックは第一容疑から外す。
- モータ回転は core1 / PWM IRQ。Wi-Fi 未登録・main の 10 Hz 送信失敗とは独立。
  「安全解除したのにテレメが無い」は因果として成立しない。

**未検証（実機ログが無い）。** 優先順:

1. 書き込み済みファームが古い（ver 1/2）か、SSID が `Benzen_new`。
   ターミナル上は cmake/build のみで flash コマンドが無い。
2. REGISTER が Pico に届いていない（経路 / 手動 IP / ARP）。
3. Pico は受けたが Mac 側が ACK/TEL を落としている（ファイアウォール）。
   この場合 USB に `#wifi: client 192.168.4.x registered` が出る。

## 対応

コードは触っていない。切り分けは USB シリアルの 2 行:

- `#wifi: AP 'Benzen_NMPC' IP 192.168.4.1 CMD:5000 TEL:5001` が無い → 書き込みファームが違うか init 失敗
- AP 行はあるが `#wifi: client ... registered` が無い → REGISTER 未着または ver 不一致で破棄
- client 行がある → Pico は登録済み。戻りパケットが Mac で落ちている

## 検証結果

未検証。USB シリアルと `ping 192.168.4.1` が次の一手。

## 次に触る人へ

- モータが回っても REGISTER なしならテレメは出ない。設計どおり。
- ver 不一致は黙って捨てる。ファームを変えたら USB の `#wifi: AP` を先に見る。
- `nmpc_udp_client.md` の ver=2 はソース（ver=3）より古い。
