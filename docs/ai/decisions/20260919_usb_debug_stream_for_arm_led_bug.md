# ディスアーム後の LED 凍結／モータ無反応を切り分けるため、USB シリアルに状態を常時垂れ流す計装を入れた

- **日付**: 2026-09-19
- **種別**: investigation
- **対象**: `Benzen_NMPC/control.cpp`（`led_control`, `loop_400Hz`, `lock_com`, `logdata_out_com`, 400Hz NMPC 出力段, 新規 `debug_stream_poll`）, `control.hpp`, `pico_copter.cpp`
- **タグ**: #Benzen_NMPC #printf #USB #stdio_usb #観測性 #LED #WS2812 #Arm_flag #LockMode #arm_gate #デバッグ計装 #一時コード

## 背景 / 症状

ユーザー報告（2026-09-19）:

> armしてモーターを回した後にもう一度disarmをしたところ、LEDがdisarmの緑点滅にはならず、
> LEDの色が消えたり飛行モードの時の色のままだったりします。LEDがおかしい時はモーターの回転も
> おかしくて、スロットルを上げてもモーターは回転しないです。そして、スロットルを上げても
> 反応しない時にもう少しスロットルを上げると急にスラスターが急回転しました。

USB 給電の卓上実験で追い込むため、USB シリアル出力を書いてほしいという依頼。

## 原因 / 判断根拠

**なぜ「修正」ではなく「計装」を先に入れたか。**

`20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md` が**同一症状を既に診断済み**で、
そこに書かれた結論は「**先に観測性を足すこと。今回の診断に必要だった状態変数が 1 つもログに出ていない**」。
その推奨は実施されないまま 8 日が経ち、同じ壁に当たった。
`Arm_flag` / `LockMode` / `g_arm_gate_ok` / `Flight_mode` / `led_control()` の分岐は
**どのログにも現れない**ので、原因を 1 つに絞れない。だから今回は観測を先に置く。

**なぜ既存の 100Hz リングログではなく USB printf か。**
100Hz ログは `angle_control()` の中で書かれ、`angle_control()` は `Arm_flag==1/2` の
`sem_release` でしか回らない（0911 で確定）。**ディスアーム後の状態は原理的に記録できない。**
今回見たい区間がまさにそこなので、リングログを拡張しても届かない。

**printf を置ける場所の制約（既存コードの前提）。**
`control.cpp:864` に既に `printf は 400Hz IRQ 内不可 → main で出力` とあり、
`g_arm2_debug_pending` という「IRQ でフラグを立て main で出す」パターンが存在する。
今回もこれに倣い、**IRQ / core1 では volatile ミラーに代入するだけ**、
出力は core0 の `main()` ループから `debug_stream_poll()` 1 か所で行う。

**`lock_com()` / `logdata_out_com()` を main から呼んではいけない。**
両者は `static uint8_t chatta` を持つデバウンス関数で、呼ぶたびに内部状態が進む。
表示のために main から呼ぶと**アーム／ログ吸い出しの状態機械そのものを壊す**。
そこで関数内で `chatta` と `state` を volatile にミラーする形にした。

**`led_control()` に付けた `else` は観測専用で、LED 出力は変えていない。**
0911 の推奨 2「末尾に `else` を足す（紫の高速点滅）」は**挙動を変える修正**なので今回は入れない。
代わりに `g_dbg_led_branch = 0` を置くだけにした。
枝 ID を全分岐に振ったので、**`led=0` が出た瞬間が「WS2812 に一切書かずに抜けた＝LED 凍結」**の直接証拠になる。

## 対応

`NMPC_DEBUG_STREAM`（`control.cpp`）1 個で ON/OFF する一時計装を追加した。**原因究明後に 0 にする。**

出力は `#dbg,...` の 1 行形式。行頭が `#` なので `tools/` の既存 CSV パーサはコメントとして捨てる。
**離散状態が 1 つでも変わった瞬間に 1 行**、変化が無くても 200 ms ごとに 1 行。
10Hz の定期サンプルだけだと遷移が取りこぼされるため、エッジ検出を主・定期を従にした。

観測している「今回の診断に必要だが従来どこにも出ていなかった」量:

| 記号 | 意味 |
|---|---|
| `led` | `led_control()` の分岐 ID。**`0` = どの枝にも当たらず LED 凍結**（本命） |
| `out` | 400Hz 出力段の経路。`1`=failsafe `2`=離陸ゲート閉 `3`=スロットル下 `4`=出力 `5`=OverG |
| `arm`/`lock` | `Arm_flag` / `LockMode` |
| `gate`/`ghold`/`gwait` | `g_arm_gate_ok` と待ち時間（**急回転の疑い先**） |
| `lk`/`dm` | `lock_com` / `logdata_out_com` の `chatta:state`（`50:1` で成立） |
| `logout` | `Logoutputflag`（`Arm_flag==4` に落ちた証拠） |

**採用しなかった案**:
- ベンチログ (10Hz CSV) の列追加 — `NMPC_BENCH_SERIAL_LOG` は既定 0 で、有効化すると
  CSV が混ざって読みにくい。今回は「状態遷移だけを人が目で追う」用途なので専用行の方が速い。
- Wi-Fi テレメトリ — USB 給電の卓上実験という前提なので USB CDC で足りる。AP 接続の手間が増えるだけ。

## 検証結果

**ビルドのみ確認。実機未検証。**

```
cd Benzen_NMPC/build && make -j8   →  [100%] Built target pico_copter
arm-none-eabi-size: text 686560 (計装前 684560, +2000 B)  bss 422436
```

⚠ **症状の再現はまだ見ていない。** どの状態を踏むかはこのログを取ってから判断する。

## 次に触る人へ

- 見かた（デバイス名は `ls /dev/cu.usbmodem*` で取り直す。詳細は
  [`20260831_minicom_usb_cdc.md`](20260831_minicom_usb_cdc.md)）:
  `minicom -D /dev/cu.usbmodem141101 -b 115200 -8 -o -C dbg.txt`
- ⚠ **`stdio_usb` は USB ホストが繋がっているのに読まれないと printf がブロックしうる**
  （SDK 既定タイムアウト 0.5 s）。止まるのは core0 の main だけで 400Hz IRQ と core1 は走るが、
  **端末を開かずに PC の USB ポートから給電するのは避けること。** モバイルバッテリ給電なら影響しない。
- 計装を消すときは `NMPC_DEBUG_STREAM` を `0` にすれば `debug_stream_poll()` は空になる。
  ミラー代入（`g_dbg_*`）は残るが副作用が無いので放置してよい。
- **`led_control()` の `else` は観測専用のまま。** 実際の修正（`else` で異常色を出す、
  `Arm_flag==4` 専用の枝、ゲート開放条件に低スロットルを加える）は
  [`20260919_disarm_led_freeze_is_armflag4_and_gate_open_throttle_jump.md`](20260919_disarm_led_freeze_is_armflag4_and_gate_open_throttle_jump.md)
  に候補として残してある。**このログで原因を 1 つに絞ってから入れる。**
- 関連: [`20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md`](20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md)
  （同一症状・観測性の推奨が未実施だった）、
  [`20260916_log_ch7_servo_ch4_and_linetrace_retired.md`](20260916_log_ch7_servo_ch4_and_linetrace_retired.md)
  （`logdata_out_com()` のガードが `Chdata[LOG]` になった経緯。`dm` 列で生値を確認できる）
