# HOVERING スイッチのモード分岐を削除して `Flight_mode` のデッドゾーンを潰し、状態機械 4 変数をログに出した

- **日付**: 2026-09-11
- **種別**: bugfix
- **対象**: `Benzen_NMPC/control.cpp`(Flight_mode 更新, NmpcBenchSnapshot, debug_print_nmpc_bench), `tools/plot_nmpc_log.py`
- **タグ**: #Benzen_NMPC #Flight_mode #MODE_FTC #LED #デッドゾーン #観測性 #ベンチログ #LockMode #arm_gate

## 背景 / 症状

[`20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md`](20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md)
で、0911_v7 の「ディスアームしても LED の色が変わらない」の候補機構として
**`Flight_mode` が `MODE_FTC` に居座る経路**を特定した。その対処。

ユーザの方針: **HOVERING モードは使っていない。運用しているのは `NORMAL` と FTC スイッチだけ。**

## 原因 / 判断根拠

旧コード（control.cpp の `Flight_mode` 更新）:

```c
if      (Chdata[HOVERING] < 200) { Flight_mode = NORMAL; ... }
else if (Chdata[HOVERING] > 500) { Flight_mode = MODE_HOVERING; ... }
// 200-500 は どちらにも当たらず Flight_mode を一切書かない
```

直後の FTC ブロックは `if (Ftc_mode) Flight_mode = MODE_FTC;` と**書くだけ**で、
OFF 時の復帰を「上の分岐が毎周期 `NORMAL` を書き直すこと」に依存していた（当時のコメントに明記）。

**HOVERING スイッチがデッドゾーン 200-500 に居ると、その前提が崩れる。**
FTC を切っても `Flight_mode == MODE_FTC` が残り、`Ftc_mode == 0` になるため
`led_control()` の 3 つの枝（`Arm_flag==2 && Ftc_mode` / `== NORMAL` / `== MODE_HOVERING`）の
**どれにも当たらない。`led_control()` には `else` が無い**ので WS2812 は一度も書かれず、
**直前の色（normal）のまま固まる**。一方 `board_led_request()` は `Arm_flag` 各分岐から呼ばれ続けるため、
**「色は変わらないがオンボード LED だけ変化する」**という報告そのものになる。

## 対応

### (3) HOVERING スイッチによるモード分岐を削除

`Flight_mode = NORMAL;` を**無条件**に実行するようにした（`Red_flag` / `Rocking_timer` のクリアも踏襲）。
`else if (Chdata[HOVERING] > 500)` の枝はコメントとして残し、復活させる場合の注意を添えた。

これで**到達し得る `Flight_mode` は `NORMAL` と `MODE_FTC` の 2 つだけ**になり、
`Ftc_mode==1` は FTC の枝、`Ftc_mode==0` は `NORMAL` の枝で必ず拾われる。
**`led_control()` に `else` を足さなくても、`Flight_mode` 起因の取りこぼしは閉じた。**

### (1) 状態機械 4 変数をベンチログ（10Hz）に追加

`flight_mode` / `lock_mode` / `gate`(=`g_arm_gate_ok`) / `ch_ftc`(=`Chdata[FTC]` 生値) を
**行末に追記**した（既存の列位置を動かさないため）。`ftc`(=`Ftc_mode`) は既にあった。
ヘッダに `#flight_mode:` `#lock_mode:` `#gate:` の凡例も追加。

**v7 の診断に必要だった状態変数が 1 つもログに出ていなかった**のが、この修正の動機である。

### 付随: `tools/plot_nmpc_log.py` を `#cols:` 動的読みに変更

従来はスクリプト側の `COLUMNS` 固定リストで、**多い列は `row[:n_cols]` で黙って捨てていた**。
`#cols:` 行があればそれを優先するようにした（無ければ従来の固定リストにフォールバック）。
これをやらないと、せっかく足した 4 列が解析側に届かない。

**採用しなかった案**:
- **`led_control()` に `else` を足す**（前回の提案 2）: 今回ユーザが選ばなかったため見送り。
  ⚠ **`Arm_flag == 4`（ログ出力状態）に枝が無い問題は残っている。** ここに入ると今も LED は固まる。
- **`Chdata[HOVERING]` を単一しきい値にする**: モード自体を使わないので、判定ごと消す方が確実。
- **`led_control()` の `MODE_HOVERING` の枝を消す**: 到達不能な死にコードになるが、
  将来 HOVERING を復活させたとき**また無言で落ちる**ので残した。
- **`unique_mission()` 内の `Chdata[HOVERING]` 参照**: 別系統の未使用関数。触っていない。

## 検証結果

- `cmake --build build_fresh -j8` 成功、**警告ゼロ**（`control.cpp` を touch して全体再コンパイル）。
  `text 682304 / bss 422364`。CRLF は 3298/3298 行で維持。
- `#cols:` の列数 **49 → 53**、`printf` の変換指定子も 53、引数も 53 で一致
  （不一致なら `-Wformat` が鳴るが鳴っていない）。
- `plot_nmpc_log.py` は構文 OK。旧ログ `0911_v7.csv`（49 列）を動的読みで正しく 49 列 337 行と解釈できることを
  同じロジックの単体再現で確認。**matplotlib が手元環境に無いためスクリプト本体の実行は未確認。**
- **実機では未検証。**

## 次に触る人へ

- **次のログで `flight_mode` / `lock_mode` / `gate` / `ch_ftc` を必ず見ること。** v7 の症状が再発したら、
  - `gate` が 0 のまま → 離陸ゲート（アームのたびに 2.5 s のカウントダウンがやり直される）
  - `lock_mode` が 3 で止まる → ディスアーム操作が完了していない
  - `flight_mode` が 2 のまま → FTC が居座っている（今回の修正で起きないはず）
  のどれかが**一目で分かる**。
- ⚠ **ベンチログは `Arm_flag==1/2` の区間しか記録できない**（`angle_control()` が `sem_release` で
  しか回らないため）。`lock_mode` が 3→0 に落ちて `Arm_flag=3` になった瞬間からログは途切れる。
  **ディスアーム後の挙動はこのログでは追えない。**
- ⚠ `Ftc_mode` 自体にも 200-500 のデッドゾーンがあるが、これは 2 位置スイッチのヒステリシスとして
  **意図的**であり、`Ftc_mode` の両値とも `led_control()` で処理されるので無害。
  ただし **3 位置スイッチを中央で放置すると FTC が ON のままラッチする**ので注意。
- 関連: [`20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md`](20260911_v7_arm_gate_recountdown_and_led_unhandled_states.md)、
  [`20260910_ru1200_loop_gain_match_and_flightmode_log_fix.md`](20260910_ru1200_loop_gain_match_and_flightmode_log_fix.md)、
  [`20260910_arm_gate_and_ekf_reinit_ru800.md`](20260910_arm_gate_and_ekf_reinit_ru800.md)
