# Benzen_new @ Pico 2 でモータ 2 個が回らない / LED 交互点滅 / ESC_calib が効かない

- **日付**: 2026-08-28
- **種別**: investigation | bugfix
- **対象**: `Benzen_new/pico_copter.cpp`, `Benzen_new/control.cpp`, `Benzen_new/pwm.cpp`（対比: `Benzen_Perfect2025_v1`）
- **タグ**: #Benzen_new #Benzen_Perfect2025 #Pico #pico2_w #RP2350 #RP2040 #ESC_calib #GOKU_HEX #F1404 #PWM #LED #WS2812 #printf #IRQ #SYS_CLK

## 背景 / 症状

実機試験（GOKU HEX 13A / T-Motor F1404 KV2900 / `drone_board_hexa_74HC245`）で 3 つの症状。

1. `Benzen_new` @ Pico 2: **GP3 と GP21 のモータが回らない**。
2. 同上: 飛行状態とアーム状態の **LED の色が交互に点滅**。
3. 同上: **`ESC_calib=1` にしてもキャリブレーションが走らない**。
   一方 `Benzen_Perfect2025` @ Pico 1 では同じ ESC・モータで全 6 個が回り、ESC_calib も動いた。

ユーザの問い: 「RP2040 → RP2350 仕様変更で何らかのエラーが起きている？」

## 原因 / 判断根拠

**確認済み事実。**

- `pico_copter.cpp:55` に `ESC_calib = 0;` があり、その直後に `pwm_init()` を呼ぶ。
  **`pwm.cpp:7` の初期値をいくら 1 にしても上書きされる。**
  ELF に `pwm.cpp` 由来の初期値 1 が入っており、`build_fresh` の UF2（15:58）は
  `pwm.cpp`（15:51）より新しい。**ビルド漏れではなく、コード構造の問題。**
  `Benzen_Perfect2025_v1/pico_copter.cpp:54` にも同じ行がある
  → ユーザが焼いた Perfect2025 は別版か、`pico_copter.cpp` 側を直したはず。**未確認。**
- **クロックは犯人ではない。** `20260828_perfect2025_v1_pico2w.md` が警告した
  「Pico 2 既定 150 MHz → 480 Hz 化」は `SYS_CLK_KHZ=125000` で既に対処済み。
  実ビルドフラグで `_Static_assert(SYS_CLK_HZ==125e6)` と
  `PLL_SYS_VCO/(POSTDIV1*POSTDIV2)==SYS_CLK_HZ` の両方が成立することを確認した。
  SDK は `SYS_CLK_HZ==125MHz` のとき POSTDIV1 を 6（既定 5）に切り替える。
- `pwm.cpp` は `Benzen_new` と `Perfect2025` で**サーボ関数以外バイト単位で同一**。
  スライス・チャンネル割り当ても同じ。**PWM 層も犯人ではない。**
- `rate_control()`（400 Hz の PWM IRQ から呼ばれる）に**有効な `printf` が 2 つ**ある。
  うち 1 つは無条件。ELF に `"REDCIRCLE"` 文字列が存在し、`Perfect2025` の ELF には無い
  → **`Benzen_new` 固有の退行**。pico-sdk の `stdio_usb` は IRQ からの呼び出しを想定していない。
- `Flight_mode` の判定条件が `Perfect2025` から変わっている。
  `Benzen_new` は NORMAL に入るのに `Chdata[REDCIRCLE] > 500` を要求し、`< 200` では REDCIRCLE へ入る。
  200–500 は前回値保持。`Perfect2025` は該当分岐が全てコメントアウトされ実質 NORMAL 固定。
- `led_control()` で 2 色交互になる分岐は 5 つあり、いずれも `Benzen_new` 固有。

**推測（未検証）。**

- GP3 = ML、GP21 = RL で**どちらも左側**。`P_com<0` かつ `Q_com<0` のとき最も duty が下がる 2 個と一致する。
  ただし静止・スティック中立なら `P_com≈0`（`Ti=100000` で積分ほぼ無効）なので、
  ミキサ偏りだけでは説明しきれない。
- より整合的なのは **ESC 未キャリブレーション**。`motor_stop()` の実出力は 1108 µs で、
  未キャリブレーション ESC（既定 ≈1040–1960 µs）では指令約 7 %。起動しきい値の真上にあり、
  どのチャンネルが「じわ回り」するかは個体差で決まる。

**比較実験が交絡している。** プログラムとチップが同時に変わっており、この 2 点だけでは切り分け不能。
`Benzen_pico1`（`Benzen_new` の RP2040 移植、`20260826_benzen_pico1_rp2040.md`）が既にあるので、
これを Pico 1 に焼けば「プログラム由来か、チップ由来か」を 1 変数で決められる。

## 対応

分析資料 [`docs/20260828_benzen_new_pico2_motor_led_analysis.md`](../../20260828_benzen_new_pico2_motor_led_analysis.md) を作成。
**コード修正は未適用**（ユーザ確認待ち）。優先修正は 2 点。

1. `pico_copter.cpp:55` の `ESC_calib = 0;` を削除し、設定箇所を 1 つにする。
2. `rate_control()` 内の `printf` 2 つを削除。

**却下した案**: 「RP2350 の SYSCLK を疑う」。最有力仮説だったが、
実ビルドフラグでの静的検証により **125 MHz で正しく動く**ことが確定したため取り下げた。
`#ifdef` 事故（`20260828_nmpc_bench_define_throttle.md`）の直後だったので同型を疑ったが、別物だった。

## 検証結果

- ELF・タイムスタンプ・`_Static_assert`・ファイル diff による静的確認は上記のとおり**実施済み**。
- **実機での検証は一切していない。** オシロによるパルス幅測定、ESC キャリブレーション実施、
  `Benzen_pico1` での切り分けはすべて**未実施**。
- LED の分岐特定には「何色と何色が交互だったか」「スイッチ位置」が必要。**未確認。**

## 次に触る人へ

- **同じ変数を 2 ファイルで設定しない。** `ESC_calib` は `pwm.cpp` と `pico_copter.cpp` の
  両方にあり、後者が黙って勝つ。今回の 3 症状のうち 1 つはこれだけが原因。
- **IRQ ハンドラの中で `printf` を書かない。** デバッグ用に一時的に入れたものが残っている。
  `rate_control()` は 400 Hz の PWM wrap IRQ から呼ばれる。
- ⚠ **記憶レイヤが分裂していた。** worktree `claude/external-memory-skills-setup-1568a5` の
  `docs/ai/decisions/` は 6 件、`main` は 25 件。同日の
  `20260828_perfect2025_v1_pico2w.md`（Pico 2 の 150 MHz 問題を既に記録していた）が
  worktree 側から見えず、調査の初手で二度手間になった。
  **worktree で作業するときは `main` 側の `docs/ai/decisions/` を grep すること。**
  本セッションで worktree にのみ存在した 3 件（NMPC スロットル分析・実装計画・decision）は
  `main` へ複製済み。
- 関連: [`20260828_nmpc_bench_define_throttle.md`](20260828_nmpc_bench_define_throttle.md)、
  [`20260828_perfect2025_v1_pico2w.md`](20260828_perfect2025_v1_pico2w.md)、
  [`20260826_benzen_pico1_rp2040.md`](20260826_benzen_pico1_rp2040.md)
