# 真因はログ吸い出しトリガの「ラッチ」。`logdata_out_com()` が 1 を返し続けて `Arm_flag` 3⇔4 を無限往復し、その間モータ停止・LED 凍結・main 停止

- **日付**: 2026-09-19
- **種別**: bugfix
- **対象**: `Benzen_NMPC/control.cpp`(`logdata_out_com`), `Benzen_NMPC/pico_copter.cpp`(main), `tools/0919_v1.txt` `0919_v2.txt` `0919_v3.txt`
- **タグ**: #Benzen_NMPC #Arm_flag #LockMode #log_output #logdata_out_com #LED #WS2812 #デバウンス #エッジ検出 #ラッチ #USB #stdio_usb

## 背景 / 症状

ユーザー報告:

> 明らかにログスイッチをONにしてOFFにしてlog_out_com()を有効化した後に上記のバグが発生します。
> log_out_com()を有効化した時しかUSBシリアルへ出力されません。変化が無ければ 200 ms ごとに
> 出力されると言われていますが、200msごとに出力はされません。

USB 給電で取得した 3 本のキャプチャ (`0919_v1/v2/v3.txt`)。

## 原因 / 判断根拠

### (1) 吸い出しが連続で何百回も走っている（ログから確定）

同一ヘッダの吸い出しブロックの反復回数:

| ファイル | `#arm_gate` が同一のまま連続した吸い出し回数 |
|---|---|
| `0919_v1.txt` | `reinit=2` が **212 回**、`reinit=0`(未アーム) が 15 回、`reinit=3` が 15 回 |
| `0919_v2.txt` | `reinit=3 wait_ms=2493` が **471 回連続** |
| `0919_v3.txt` | `reinit=16 rows=358` が 7 回、`reinit=20 rows=841` が 3 回 |

**`#arm_gate` の値が 1 度も変わらない**＝間にアームが挟まっていない＝**1 回の操作で連続して吸い出している。**

### (2) 機構（ソースから確定）

`logdata_out_com()` は `static uint8_t chatta, state` のデバウンス関数で、
スティック条件が続く限り **`state` は 1 に貼り付く**。一方 `log_output()` は末尾で
`Arm_flag = 3; Logoutputflag = 0; LockMode = 0;` と戻す。

呼び出し側は `Arm_flag==3 && LockMode==0` の中で **毎 400Hz** これを見ているので、

```
Arm_flag=4 → log_output() 完走 → Arm_flag=3 → 次の 400Hz で logdata_out_com()==1 → Arm_flag=4 → …
```

**スティックを離すまで止まらない。** ⚠ **デバウンス済みレベルをエッジトリガとして使っていたのが真因。**

### (3) 3 つの症状がこの 1 つで全部説明できる

| 症状 | 機構 |
|---|---|
| スロットルを上げてもモータが回らない | `Arm_flag==4` は毎周期 `motor_stop()` |
| LED が緑点滅にならない / 飛行色のまま | `led_control()` に `Arm_flag==4` の枝が無く WS2812 を書かずに抜ける |
| LED が消えたり点いたり | `Arm_flag==4` 自身の `rgbled_switch(led)` が約 1 秒でトグル |

### (4) ⚠ USB デバッグ出力が出なかった理由（設計ミスを 1 つ抱えていた）

`main()` の `while (Logoutputflag==1) log_output();` は **`while`**。
`Arm_flag==4` が毎 400Hz で `Logoutputflag = 1` を書き直すので、
`log_output()` が 0 にしても次の判定までに 1 に戻り、**内側ループから永久に抜けられない。**
その結果 `board_led_poll()` も `debug_stream_poll()` も止まる。

⚠ **つまり 2026-09-19 に入れた計装は、観測したかったまさにその状態を構造的に観測できなかった。**
「吸い出しを起動した時しかシリアルに出ない」という報告はこれで説明がつく。

### (5) ⚠ ただし「吸い出し前にも 1 行も出ていない」は上記では説明できない（未確定）

`0919_v3.txt` には `reinit=16` の群と `reinit=20` の群があり、**間に 4 回のアームが入っている**。
その間 `Arm_flag==2 / Logoutputflag==0` で main は自由に回っているので `#dbg` が出るはず。
`0919_v2.txt` は起動 (`#init pwm` → `#Please wait 0[s]`) を捉えているが、
**直後に出るはずの `#dbg_cols:` が無い。**

`build/` `build_fresh/` 双方の ELF に `dbg_cols` 文字列と `debug_stream_poll` シンボルがあり、
`main` の逆アセンブルでも `bl <_Z17debug_stream_pollv>` がループ内に出ている。
→ **ビルド成果物は正しい。書き込まれたファームが古かった可能性が高いが、断定はできない。**
判別法: **起動時の `#Please wait 0[s]` の直後に `#dbg_cols:` が出るかどうか。**

## 対応

**採用しなかった案: 本番時に LOG 関連の分岐を全部コメントアウトする（ユーザー提案）。**
却下理由:
1. **リングログは実機解析の唯一の計器**で、FTC の検討はすべてこの CSV に依存している。
   3 行のラッチ不具合を避けるために計器を捨てるのは代償が釣り合わない。
2. **欠陥が消えない。** 「デバウンス済みレベルをエッジとして使う」という同じ形が
   `lock_com()` にもある（あちらは `LockMode` が前進するので露呈していないだけ）。
3. 吸い出しを消すと**基板からデータを取り出す手段が無くなる。**

**修正 A（真因）**: `logdata_out_com()` を**立ち上がり (0→1) でのみ 1 を返す**形にした。
呼び出しは `loop_400Hz()` の 1 か所だけなのでエッジを取りこぼす心配は無い。
「1 回の操作で 1 回だけ吸い出す」という本来の意図に一致する。

**修正 B（観測性）**: `main()` の `while (Logoutputflag==1)` を **`if`** にした。
`log_output()` は 1 呼び出し 1 行の設計なので `if` でも吸い出しは進み、
**吸い出し中も `board_led_poll()` / `debug_stream_poll()` が回る。**

**今回入れなかったもの**（別件・別の判断が要る）:
- `led_control()` への `Arm_flag==4` 専用の枝 — 修正 A で state 4 は 1 回の吸い出し分しか続かなくなる。
  また state 4 は自前で `rgbled_switch()` を叩くので led_control 側に枝を足しても上書きされる。
- 離陸ゲート開放時の低スロットル要求（急回転の別機構）。
  [`20260919_disarm_led_freeze_is_armflag4_and_gate_open_throttle_jump.md`](20260919_disarm_led_freeze_is_armflag4_and_gate_open_throttle_jump.md) 参照。

## 検証結果

- 反復回数はログから確定（212 / 471 / 7 / 3 回）。**機構はソースから確定。**
- ビルド: `text 686560 → 686600`。**実機未検証。**
- ⚠ (5) の「古いファームだったのか」は未確定のまま。次回の起動ログで判定する。

## 次に触る人へ

- **起動時に `#dbg_cols:` が出ているか必ず確認してから実験すること。** 出ていなければ古いファーム。
- ⚠ **`while (Logoutputflag==1)` の形に戻さないこと。** main が吸い出しに閉じ込められ、
  LED ポーリングもデバッグ出力も止まる。
- ⚠ **デバウンス済みの `state` をエッジトリガとして使わないこと。** 同じ形が `lock_com()` にある。
- 関連: [`20260919_usb_debug_stream_for_arm_led_bug.md`](20260919_usb_debug_stream_for_arm_led_bug.md)（この計装の導入）、
  [`20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md`](20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md)
  （0911 時点で `Arm_flag==4` は「入って抜けている」と書いたが、**何度も入り直していたことまでは掴めていなかった**）
