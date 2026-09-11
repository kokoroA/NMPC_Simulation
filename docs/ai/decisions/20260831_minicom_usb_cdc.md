# Pico USB CDC を見る minicom コマンド

- **日付**: 2026-08-31
- **種別**: investigation
- **対象**: `/dev/tty.usbmodem141101`（Pico 2 W `stdio_usb`）
- **タグ**: #Benzen_NMPC #Pico #USB #シリアル #minicom #stdio_usb

## 背景 / 症状

`20260831_benzen_nmpc_register_ack_none.md` の次の一手が USB シリアル。
リポジトリに minicom 手順は無く、`docs/benzen_nmpc_build.md` は「USB で `#wifi: AP` を見る」とだけ書いてある。

## 原因 / 判断根拠

- `pico_enable_stdio_usb=1` / `stdio_uart=0`。物理 UART ではなく USB CDC。
- ボーレートはホスト側の慣習値。Pico の CDC は 115200 を解釈しない。不一致で文字化けは起きない。
- macOS の `/dev/tty.*` は DCD 待ち。USB CDC は DCD を出さないことがあり、open で止まる。
  `/dev/cu.*` は待たない。同じ番号の対が今ある（141101、2026-08-31 14:05）。
- このマシンの minicom は 2.8（Homebrew）。`minirc.dfl` は lock と escape-key のみ。
  ポート・ボー・フロー制御は未設定。minicom 既定の Hardware Flow Control = Yes だと無出力になり得る。
- 既定ビルドは `NMPC_BENCH_SERIAL_LOG=0`。USB に CSV は出ない。見るのは `#wifi:` の 2 行。

## 対応

推奨（このマシンで確認したデバイス名）:

```bash
minicom -D /dev/cu.usbmodem141101 -b 115200 -8 -o
```

ユーザー指定の `tty` でも開けるが、DCD 待ちで止まるなら `cu` に替える。

初回だけ Ctrl-A（このビルドは Escape/Meta の可能性あり）→ `O` → Serial port setup
で Hardware Flow Control を No。`pu rtscts No` を `~/.minirc.dfl` に書けば次回から不要。

終了はコマンドキー + `Q`。キャプチャは `-C wifi_serial.txt`。

## 検証結果

- デバイス実在: `tty.usbmodem141101` と `cu.usbmodem141101`。
- minicom 2.8 は `/usr/local/bin/minicom`。対話接続は未実施。
- `stty -f /dev/cu.usbmodem141101`: 8N1、`-crtscts`、`-ixon`。

## 次に触る人へ

- 番号 `141101` は USB ポートで変わる。`ls /dev/cu.usbmodem*` で取り直す。
- 見る行: `#wifi: AP 'Benzen_NMPC' ...` と `#wifi: client ... registered`。
  解釈は `20260831_benzen_nmpc_register_ack_none.md`。
