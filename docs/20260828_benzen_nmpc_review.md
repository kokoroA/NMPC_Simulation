# Benzen_NMPC 現状レビュー — 残っている修正点

- **日付**: 2026-08-28
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/`（本日のスロットル修正適用後）
- **契機**: ESC キャリブレーション後、実機で全 6 モータが正常に回ることを確認
- **関連**: [`20260828_hexa_motor_test_analysis.md`](20260828_hexa_motor_test_analysis.md)、
  [`20260828_nmpc_throttle_fix_plan.md`](20260828_nmpc_throttle_fix_plan.md)、
  decisions [`20260828_nmpc_bench_define_throttle.md`](ai/decisions/20260828_nmpc_bench_define_throttle.md)、
  [`20260828_benzen_new_pico2_motor_led.md`](ai/decisions/20260828_benzen_new_pico2_motor_led.md)

---

## 0. まず確認できたこと（良好）

| 項目 | 状態 |
|---|---|
| 本日の修正の反映 | `build_fresh/pico_copter.uf2`（15:49）に反映済み。ELF に `0.2f*7.4` = **0 個**、`0.5f*7.4` = **1 個**、`T_ref_coe` = **0.6** |
| リアルタイム経路の `printf` | **400 Hz IRQ・100 Hz core1 ともに有効な `printf` は 1 つも無い**。`Benzen_new` に残っている 2 つのような退行は無い |
| SYSCLK / PWM | 125 MHz / 400 Hz / 1108–1876 µs |
| SBUS パーサのバッファ | `sbus_data[25]`、`chars_rxed` は 25 で 0 に戻る。**範囲外アクセスは無い** |
| メモリ | text 674,224 / bss 402,456（SRAM 520 KB の約 77 %）+ core0 スタック 32 KB |

**ESC キャリブレーションで直った**という結果は、
[`20260828_benzen_new_pico2_motor_led.md`](ai/decisions/20260828_benzen_new_pico2_motor_led.md) の仮説
（出力下限 1108 µs が ESC の起動しきい値の真上にあった）の裏付けになります。

---

## 1. 【最優先・安全】飛行前に直すべき 3 点

### S1. 受信機のフェイルセーフが存在しない

`radio.cpp` は SBUS を割り込みで読み、**フラグバイトを取得までしている**:

```cpp
Chdata[16] = sbus_data[23];   // ← SBUS のフラグバイト。bit2=frame lost, bit3=failsafe
```

**しかしこの値はどこからも参照されていない。** 受信途絶のタイムアウトも無い。

→ **プロポの電源が切れる／リンクが切れると、`Chdata[]` は最後の値のまま固まり、
モータは最後のスロットルで回り続ける。**

```bash
grep -rn "failsafe\|timeout\|last_recv" radio.cpp control.cpp   # ヒット 0 件
```

**対応（最小）**: `sbus_data[23] & 0x08`（failsafe）で `motor_stop()`。
`sbus_data[23] & 0x04`（frame lost）が連続したらカウントして停止。
併せて「最後に有効フレームを受けた時刻」を持ち、**200 ms 途絶で停止**。

### S2. 過大 G 保護が数値的に発火しない

`control.cpp` の `sensor_read()`:

```cpp
acc_norm = sqrt(Ax*Ax + Ay*Ay + Az*Az);
if (acc_norm > 800.0) OverG_flag = 1;
```

`Ax..Az` の単位は **m/s²**（`acceleration_mg[] * GRAV * 0.001`）。
`800 m/s² ≈ 81.5 g` である。

一方 `sensor.cpp:79` は `lsm9ds1_xl_full_scale_set(&Imu_h, LSM9DS1_16g)`。
**センサの最大出力は 16 g = 157 m/s²、3 軸合成でも約 272 m/s² が上限。**

→ **`OverG_flag` は絶対に 1 にならない。** 墜落・激突時のモータ停止が効かない。

**対応**: 閾値を 100–150 m/s²（約 10–15 g）程度へ。あるいは
コメントアウトされている角速度判定（`rate_norm > 70.0` [rad/s]）も見直す。
**閾値を下げると誤検知で飛行中に停止するので、まず係留でログを取って分布を見ること。**

### S3. `ESC_calib` を 2 か所で設定しており、`pwm.cpp` 側が必ず負ける

```cpp
// pwm.cpp:7
uint8_t ESC_calib = 1;          // ← 15:49 に 1 へ変更されている

// pico_copter.cpp:55  (2026-08-13 以降 未編集)
ESC_calib = 0;                  // ← 無条件に 0 へ戻す
pwm_init();                     // ← その直後
```

**現在焼かれているファームを逆アセンブルして確認済み**（`build_fresh/pico_copter.elf`）:

```
10000282:  strb r2, [r3, #0]      ; r2=0, r3=0x20002920 (= ESC_calib)
10000284:  bl   100054e0 <pwm_init>
```

→ **このファームは ESC キャリブレーションを実行できない。**

**質問 Q1**: 今回のキャリブレーションはどうやって実施しましたか。
考えられるのは「以前に別ファーム（Perfect2025 等）で校正済みで、**ESC 側が校正値を保持している**」です。
校正値は ESC の不揮発メモリにあるので、一度合わせれば以後どのファームでも効きます。

⚠ **どちらにせよ、ESC を交換・リセットしたら同じ症状が再発し、
現在のコードでは校正できません。** `pico_copter.cpp:55` を消して設定箇所を 1 つにしてください。
（`Benzen_new` / `Perfect2025` も同じ構造です。）

---

## 2. 【制御の正しさ】初飛行の挙動に直接効く

### C1. `T_max = 2.04 N` がモデルと実機で整合していない可能性が高い

`nmpc_model.cpp` の入力行列は **`T_max` に正比例**する:

```cpp
B_c(2,k) = -kYm[k] * T_max / Ix;   // roll
B_c(3,k) =  kXm[k] * T_max / Iy;   // pitch
B_c(4,k) =  kS[k] * C_QT * T_max / Iz;  // yaw
```

- モデル上のホバー duty = `T_hover_per_motor / T_max` = `0.5821 / 2.04` = **0.285**
- 実機のホバー duty（= `T_ref × T_ref_coe`）は**未測定**

**`T_max` を過小に見積もると、NMPC は自分の操作権限を過小評価し、指令を出しすぎる。**
ユーザ自身が「T_max はおそらくもっと大きい」と述べており、その場合 **発振側に倒れる**。

| 実測ホバー duty | 逆算した `T_max` |
|---|---|
| 0.15 | 3.88 N |
| 0.20 | 2.91 N |
| 0.24 | 2.43 N |
| 0.285 | 2.04 N（現行値と一致） |

**対応**: 係留またはホバー直前まで上げて `T_ref × T_ref_coe` を実測し、
`T_max = 0.5821 / hover_duty` で更新する。`nmpc_params.hpp:44` のコメントどおりの手順。

### C2. `nmpc_params.hpp:41` のコメントが古い

```cpp
constexpr float T_hover_per_motor = mass * gravity / 6.0f; // ≈ 0.4905
```

`0.356 × 9.81 / 6 = 0.5821` であり、**0.4905 ではない**（`mass = 0.30 kg` 時代の値）。
C1 の逆算をするときにこの数字を使うと 19 % ずれる。**コメントを修正すること。**

### C3. ADMM の box 下限と実機のクランプが食い違っている

| | duty の下限 |
|---|---|
| ソルバ（`delta_u_bounds`: `du_min = -u_hover`） | **0.0** |
| 実機（`rate_control` の `minimum_duty = Disable_duty`） | **0.1** |

`duty < 0.1` の領域は**ソルバが「出せる」と思っているのに実際は出ない**。
1 発故障時など、あるモータを絞り切りたい場面で最適解と実際の入力がずれる。

**対応**: `delta_u_bounds` の下限を `Disable_duty - u_hover` にする（`Disable_duty` を
`nmpc_params.hpp` 側へ移すか、引数で渡す）。

### C4. ゲート閾値にヒステリシスが無い

- NMPC 実行: `T_ref * T_ref_coe >= Flight_duty (0.05)`
- モータ出力: `T_ref * T_ref_coe >= Disable_duty (0.1)`

閾値近傍でスロットルが震えると、**warm start の破棄（`nmpc_reset_request`）と
姿勢ゼロ点の再取得（`Phi_bias = Phi` 等）が繰り返される。**
離陸直前がちょうどこの領域なので、実害が出やすい。

**対応**: 立ち上がり／立ち下がりで別閾値を持たせる（例: ON 0.10 / OFF 0.08）。

---

## 3. 【運用・保守】飛行前でなくてよいが直しておきたい

| # | 内容 |
|---|---|
| **O1** | `#ifdef NMPC_WIFI_TELEM` が残っている。今は `=1` なので動くが、**本日直した事故と同型**。`#if` へ統一する |
| **O2** | `Flight_mode` 判定が `Chdata[REDCIRCLE] > 500` を要求。スイッチ位置によっては NORMAL に入れず、`< 200` では REDCIRCLE に入る。200–500 は前回値保持。`i2c_connect` は 1 固定なので条件としては無意味 |
| **O3** | `motor_stop()` の実出力は **1108 µs**（`set_duty_*()` の `duty<0.01 → 0.01` クランプ）。真の停止ではない。ESC 校正後なら実害は小さいが、`motor_stop()` だけは DUTYMIN 未満を出せるようにするのが素直 |
| **O4** | SBUS フレームの終端バイト（`sbus_data[24] == 0x00`）を検証していない。壊れたフレームをそのまま採用する |
| **O5** | core1 スタックは 4 KB（`PICO_CORE1_STACK_SIZE=0x1000`）。`nmpc_step` は BSS へ逃がしてあるが、**実使用量は未測定**。スタックを既知パターンで埋めて高水位を測っておくと安心 |
| **O6** | `NMPC_BENCH_COST=0` にしたことによる `iter_us` の短縮量が**未測定**。未解決課題「周期 10 ms 以内」に効く可能性がある |

---

## 4. 推奨する着手順

| # | 作業 | 理由 |
|---|---|---|
| 1 | **S1 フェイルセーフ** | 飛ばす前提が崩れる。フラグバイトは既に取得済みなので実装は小さい |
| 2 | **S3 `ESC_calib` の一本化** | ESC 交換で必ず再発する。1 行削除 |
| 3 | **S2 OverG 閾値** | まず係留でログを取り、分布を見てから閾値を決める |
| 4 | **C1 `T_max` 実測** | 初飛行の安定性に直結。ホバー duty を測るだけ |
| 5 | C2 / C3 / C4 | C1 の実測と同時に |
| 6 | O1 / O4 / O5 / O6 | 飛行後でよい |

---

## 5. 質問

| # | 質問 | なぜ必要か |
|---|---|---|
| **Q1** | 今回の ESC キャリブレーションはどう実施しましたか（現在のファームは実行できないことを逆アセンブルで確認済み）。以前に別ファームで校正した値が ESC 側に残っていた、で合っていますか | S3 の対応方針が変わる |
| **Q2** | 機体の実重量は 0.356 kg で合っていますか（`nmpc_params.hpp` の `mass`） | C1 の逆算に直接効く |
| **Q3** | S1（フェイルセーフ）と S2（OverG 閾値）を実装しますか。実装するなら閾値の決め方（係留ログを先に取るか）を決めたいです | 飛行前の最優先事項 |
| **Q4** | ホバー duty の実測はいつ行いますか。係留で `T_ref × T_ref_coe` を読めれば `T_max` が確定します | C1 |

---

## 付録: 確認方法

| 主張 | 確認手段 |
|---|---|
| リアルタイム経路に printf が無い | `led_control` / `loop_400Hz` / `servo_control` / `rate_control` / `angle_control` / `logging` / `sensor_read` / `kalman_filter` の各行範囲で有効行の `printf` を数え、すべて 0 |
| 今日の修正がファームに入っている | ELF のバイト走査（`0.2f*7.4` = 0、`0.5f*7.4` = 1、float `0.6` = 1、float `0.7` = 0） |
| `ESC_calib` が 0 で上書きされる | `main()` の逆アセンブル。`strb r2,[r3]`（r2=0, r3=0x20002920=`ESC_calib`）→ `bl pwm_init` |
| OverG が発火しない | `sensor.cpp:79` の `LSM9DS1_16g` と閾値 800 m/s² の比較 |
| フェイルセーフが無い | `radio.cpp` / `control.cpp` を `failsafe|timeout|last_recv` で grep → 0 件。`Chdata[16]` への代入はあるが参照が無い |
| SBUS バッファが安全 | `uint8_t sbus_data[25]`、書き込みは 0..24、`chars_rxed==25` でリセット |
