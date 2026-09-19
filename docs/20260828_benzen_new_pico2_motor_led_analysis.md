# Benzen_new × Pico 2 でモータ 2 個が回らない / LED が交互点滅 / ESC_calib が効かない — 原因分析

- **日付**: 2026-08-28
- **供試体**: `drone_board_hexa_74HC245`、GOKU HEX 13A（6-in-1 ESC）、T-Motor F1404 KV2900
- **比較**: `Benzen_new` @ Raspberry Pi Pico 2 ⇔ `Benzen_Perfect2025` @ Raspberry Pi Pico（1）
- **関連**: [`20260828_hexa_motor_test_analysis.md`](20260828_hexa_motor_test_analysis.md)、
  decisions [`20260828_perfect2025_v1_pico2w.md`](ai/decisions/20260828_perfect2025_v1_pico2w.md)、
  [`20260825_benzen_new_pico2w.md`](ai/decisions/20260825_benzen_new_pico2w.md)、
  [`20260826_benzen_pico1_rp2040.md`](ai/decisions/20260826_benzen_pico1_rp2040.md)

---

## 0. 結論

| # | 症状 | 判定 | 確度 |
|---|---|---|---|
| **A** | `ESC_calib=1` にしても ESC キャリブレーションが走らない | **ソフト。`pico_copter.cpp` が `pwm.cpp` の設定を無条件に上書きしている。`pwm.cpp` を何度書き換えても絶対に効かない** | **確定** |
| **B** | GP3 / GP21 のモータが回らない | **A の帰結が最有力。** ESC が未キャリブレーションのまま、出力下限 1108 µs が起動しきい値の真上にある。加えて GP3/GP21 は**ミキサ上どちらも左側**で、姿勢指令が乗ると最も duty が下がる 2 個 | 高（要実測） |
| **C** | LED が 2 色交互に光る | **ソフト。`Benzen_new` で追加/変更された `Flight_mode` 判定と LED 分岐。** `Perfect2025` には該当分岐が存在しない | 高（色の確認要） |
| **D** | 「RP2040 → RP2350 移行のエラーでは？」 | **クロックに関しては否。** `SYS_CLK_KHZ=125000` は RP2350 でも実際に PLL を 125 MHz にすることを静的に確認済み。PWM は Pico 1 と同一の 400 Hz / 1100–1900 µs | **確定（否定）** |

> **今回の比較は交絡している。** プログラム（`Benzen_new` ⇔ `Perfect2025`）と
> チップ（RP2350 ⇔ RP2040）が**同時に**変わっている。
> どちらが効いているかは、この 2 点だけでは原理的に切り分けられない。§5 に切り分け手順を示す。

---

## 1. 【確定】ESC_calib は構造的に実行不可能

`Benzen_new/pwm.cpp:7`

```cpp
uint8_t ESC_calib = 1;      // ← ユーザが 1 に変更した
```

`Benzen_new/pico_copter.cpp:53-56`

```cpp
  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib = 0;            // ← ここで無条件に 0 へ戻される
  pwm_init();               // ← その直後に呼ばれる
```

`pwm_init()` は `if (ESC_calib == 1)` を見るので、**キャリブレーション経路には決して入らない。**
`pwm.cpp` の初期値をいくら変えても意味がない。

### 確認

- ビルド成果物 `build_fresh/pico_copter.elf` に `ESC_calib` シンボルが実在し、
  `pwm.cpp` 由来の初期値 1 が入っている（= `pwm.cpp` の変更はビルドに反映されている）。
- `build_fresh/pico_copter.uf2` は **2026-08-28 15:58**、`pwm.cpp` は **15:51**。
  **ビルドが古いのではない。** 再ビルドは正しく行われている。
- したがって残る説明は `pico_copter.cpp:55` の上書きのみ。

### ⚠ `Benzen_Perfect2025_v1` も同じ構造になっている

`Benzen_Perfect2025_v1/pico_copter.cpp:54` にも `ESC_calib= 0;` がある。
**このリポジトリ内の Perfect2025 でも、そのままではキャリブレーションは走らない。**

→ **質問 Q2**: Perfect2025 側では `pico_copter.cpp` の方を書き換えましたか。
それとも別のコピー（`Benzen_Perfect2025_v1.zip` など）を焼きましたか。
ここが分かると A と B の因果が確定します。

### 修正

```diff
-  ESC_calib = 0;
+  // ESC_calib は pwm.cpp の初期値をそのまま使う（ここで上書きしない）
```

または `pico_copter.cpp` 側を唯一の設定箇所にする。**2 箇所で同じ変数を設定するのをやめること。**

> ⚠ キャリブレーションは起動直後に全 6 ch へ **DUTYMAX（1900 µs）を 3 秒間**出す。
> **必ずプロペラを外して実施すること。** また DUTYMIN 保持が 1 秒しかなく、
> ESC が学習し切らない可能性がある（2 秒以上が無難）。

---

## 2. 【確定】RP2350 移行でクロックは壊れていない

decision [`20260828_perfect2025_v1_pico2w.md`](ai/decisions/20260828_perfect2025_v1_pico2w.md) が指摘したとおり、
**Pico 2 の既定 SYSCLK は 150 MHz** で、`wrap=3124 / clkdiv=100` の 400 Hz は 125 MHz 前提である。
放置すれば制御周期が 480 Hz、パルスが 917–1583 µs になり、まさに今回のような症状を起こす。

**しかしこれは既に対処済みだった。** 両者の `CMakeLists.txt` に `SYS_CLK_KHZ=125000` がある。

### 確認（実際のビルドフラグでコンパイルして検証）

`Benzen_new/build_fresh` の `flags.make` から実際の `-D` と `-I` を取り出し、次を通した:

```c
_Static_assert(SYS_CLK_HZ == 125000000, "SYS_CLK_HZ != 125MHz");
_Static_assert(PLL_SYS_VCO_FREQ_HZ/(PLL_SYS_POSTDIV1*PLL_SYS_POSTDIV2) == SYS_CLK_HZ, "PLL != SYS_CLK");
```

**両方成立。** SDK の `hardware/clocks.h` は

```c
#ifndef PLL_SYS_POSTDIV1
#if SYS_CLK_HZ == 125 * MHZ
#define PLL_SYS_POSTDIV1  6      // ← 125 MHz 指定ならこちら
#else
#define PLL_SYS_POSTDIV1  5      // ← RP2350 既定の 150 MHz
#endif
#endif
```

となっており、VCO 1500 MHz / 6 / 2 = **125 MHz**。RP2350 でも指定どおり動く。

→ **PWM は 400 Hz、パルス 1100–1900 µs（`motor_stop()` は 1108 µs）。Pico 1 と完全に同一。**
**クロックは犯人ではない。**

### `pwm.cpp` も犯人ではない

`Benzen_new/pwm.cpp` と `Benzen_Perfect2025_v1/pwm.cpp` を比較したところ、
**差分はサーボ関数（`payload_half`）と `SERVO_HOOK/RELEASE` の値だけ**で、
スライス番号・チャンネル割り当て・`set_duty_*()`・E9 workaround はバイト単位で同一。

---

## 3. GP3 / GP21 が回らない件

### 3-1. この 2 本はミキサ上どちらも「左側」

| GPIO | PWM | ファーム上のモータ | ミキサ |
|---|---|---|---|
| **GP3** | slice1 ch**B** | **ML**（Middle Left） | `common + 3.5311·P_com − 1.7655·R_com` |
| **GP21** | slice2 ch**B** | **RL**（Rear Left） | `common + 1.7655·P_com − 3.0562·Q_com + 1.7655·R_com` |

`common = T_ref · T_ref_coe`。右側 3 個（FR/MR/RR）は `P_com` の符号が逆。

**`P_com < 0`（右ロール指令）かつ `Q_com < 0`（機首下げ指令）のとき、6 個のうち最も duty が下がるのが
ちょうど ML と RL である。** 観測された 2 個と一致する。

前回報告の「GP3 と GP26」も、GP26 = Pico 物理ピン 26 = **GP20 = FL（Front Left）**と読めば
やはり左側であり、同じ系統の現象に見える。

### 3-2. ただし静止・スティック中立なら P_com ≈ 0 のはず

`p_pid` は `Kp=0.025, Ti=100000`（積分ほぼ無効）。機体が静止していれば角速度誤差 ≈ 0 なので
`P_com ≈ 0`、6 個の duty はほぼ等しくなる。

**したがって「ミキサ由来の偏り」だけでは説明しきれない。**
より整合的なのは **§1 の ESC 未キャリブレーション**である。

- `set_duty_*()` は `duty < 0.01` を 0.01 に持ち上げるため、`motor_stop()` の実出力は **1108 µs**。
- ESC が未キャリブレーション（BLHeli_S 既定 ≈ 1040–1960 µs）なら、1108 µs は **指令約 7 %**。
- F1404 KV2900 + プロペラの起動しきい値がちょうどこの付近にあり、
  **6 個中どれが「じわ回り」するかは個体差で決まる。**
- Perfect2025 側でキャリブレーションが成功していたなら、ESC のレンジが 1100–1900 µs に一致し、
  同じ ESC・同じモータでも全 6 個が素直に回る。**観測と一致する。**

### 3-3. 切り分け

| 実験 | 分かること |
|---|---|
| GP3/GP21 の ESC コネクタを他のチャンネルと**入れ替える** | 症状がモータ/ESC 側に付いて回るか、GPIO 側に留まるか |
| オシロで 6 本のパルス幅を**同時に**比較 | ファームが本当に同じ duty を出しているか（ミキサ偏りの有無） |
| プロペラを外して ESC キャリブレーションを実施 | §1 の仮説の直接検証 |

---

## 4. LED が 2 色交互に光る件

`led_control()` は **400 Hz IRQ から呼ばれ**、`static uint16_t cnt` を 0→99 で回して
`cnt==0` と `cnt==50` で色を切り替える。**周期 100/400 Hz = 0.25 秒、つまり 4 Hz の交互点滅。**

2 色交互になる分岐は次の 4 つ:

| 条件 | 色 |
|---|---|
| `Arm_flag==2 && Flight_mode==HOVERING` | normal ⇔ **pink** |
| `Arm_flag==2 && Flight_mode==AUTO_HORIZONTAL_TURN` | blue ⇔ 消灯 |
| `Arm_flag==2 && Flight_mode==AUTO_FIGURE_EIGHT` | green ⇔ blue |
| `Arm_flag==2 && Flight_mode==AUTO_CLIMBING_TURN` | pink ⇔ blue |
| `Arm_flag==3` | green ⇔ 消灯 |

**`Perfect2025` の `led_control()` にはこれらの分岐が無い**（`Flight_mode` は実質 NORMAL 固定）。
つまり LED 異常は **`Benzen_new` 固有**であり、RP2350 とは無関係の可能性が高い。

### 4-1. `Flight_mode` の判定条件が Perfect2025 から変わっている

| | NORMAL に入る条件 |
|---|---|
| **Perfect2025** | `Chdata[LOG] < 200 && Chdata[HOVERING] < 200 && i2c_connect == 1`（他モードは全てコメントアウト） |
| **Benzen_new** | `Chdata[REDCIRCLE] > 500 && Chdata[LOG] < 200 && Chdata[HOVERING] < 200 && i2c_connect == 1` |

**`Benzen_new` は NORMAL に入るのに `Chdata[REDCIRCLE] > 500` を要求する。**
さらに `Chdata[REDCIRCLE] < 200` だと **REDCIRCLE モード**へ入る（Perfect2025 では無効化されていた分岐）。
200–500 の間は**どの分岐にも入らず前回値を保持**する。

→ スイッチが「Perfect2025 のときの位置」のままだと、`Benzen_new` では
**NORMAL に入れない / REDCIRCLE に入ってしまう**。
`Chdata[REDCIRCLE]` が 200/500 付近で暴れれば **NORMAL ⇔ REDCIRCLE で色が交互に変わる。**

「飛行状態とアーム状態の色が交互」という表現からは、`Arm_flag` が **2 ⇔ 3 で往復**している線も濃い。
`Arm_flag==3` は「Disable Flight」で green ⇔ 消灯の点滅になる。

→ **質問 Q5**: 具体的に**何色と何色**が交互でしたか。また、そのときスイッチ（REDCIRCLE / LOG / HOVERING）
はどの位置でしたか。ここで分岐が一意に決まります。

### 4-2. 【確定】400 Hz 割り込みの中で `printf` を実行している

`Benzen_new/control.cpp` の `rate_control()`（= PWM wrap IRQ から呼ばれる）に、
**コメントアウトされていない `printf` が 2 つある。**

```cpp
  printf("REDCIRCLE\n");                    // REDCIRCLE モード時
  ...
  printf("%d\n", (Chdata[REDCIRCLE]));      // ← 無条件。毎周期 400 回/秒
```

- ELF 検証: `Benzen_new` の成果物に `"REDCIRCLE"` 文字列が**存在する**（`Perfect2025` には**無い**）。
- pico-sdk の `stdio_usb` は mutex を取るため、**割り込みハンドラから呼ぶことを想定していない。**
  デッドロック回避機構により出力自体は捨てられるが、**書式変換のコストは毎周期かかる。**
- `Benzen_new` はさらに Wi-Fi（`pico_cyw43_arch_lwip_threadsafe_background`）を動かしており、
  CYW43 の非同期コンテキストが割り込みを止める区間がある。`Perfect2025` は `pico_cyw43_arch_none`。

**この 2 つの `printf` は削除すべき。** 制御周期のジッタ源であり、LED の挙動にも影響し得る。

---

## 5. 切り分け実験（推奨順）

**`Benzen_pico1` が既に存在する。**（`20260826_benzen_pico1_rp2040.md`：`Benzen_new` の RP2040 移植版、
`build_fresh/pico_copter.uf2` は 2026-08-26 14:13）**これを使えば交絡を外せる。**

| # | 実験 | 判定 |
|---|---|---|
| 1 | **`Benzen_pico1` を Pico 1 に焼く**（同じ ESC・モータ・基板） | GP3/GP21 が回らなければ **プログラム側**。回れば **チップ/移植側** |
| 2 | `pico_copter.cpp:55` を消して `ESC_calib=1` でキャリブレーション（プロペラ無し） | 全 6 個が同じスティック位置で回り始めるか |
| 3 | `rate_control()` の `printf` 2 つを削除して再ビルド | LED とタイミングが安定するか |
| 4 | オシロで 6 本のパルス幅を比較（0→100 % 掃引） | ミキサ偏りの有無。1108 → 1860 µs を連続に掃くか |
| 5 | GP3/GP21 の ESC コネクタを他チャンネルと入れ替え | 症状が GPIO 側かモータ/ESC 側か |

**1 → 2 → 3 の順で 1 つずつ変える。** 同時に変えると今回と同じ交絡が再発する。

---

## 6. 修正候補（本資料では未適用）

| # | 対象 | 内容 | 優先 |
|---|---|---|---|
| M1 | `Benzen_new/pico_copter.cpp:55` | `ESC_calib = 0;` を削除。設定箇所を 1 つにする | **高** |
| M2 | `Benzen_new/control.cpp` | `rate_control()` 内の `printf` 2 つを削除（IRQ 内 printf は禁止） | **高** |
| M3 | `Benzen_new/pwm.cpp` | ESC_calib 時の DUTYMIN 保持を 1 s → 2 s 以上へ | 中 |
| M4 | `Benzen_new/control.cpp` | `Flight_mode` 判定の `Chdata[REDCIRCLE]` 条件を意図どおりに整理 | 中 |
| M5 | `Benzen_new/pwm.cpp` | `set_duty_*()` の `duty<0.01 → 0.01` クランプを見直し、`motor_stop()` は真のゼロを出す | 中 |

---

## 7. 質問

| # | 質問 | なぜ必要か |
|---|---|---|
| **Q1** | 焼いた Perfect2025 は具体的にどれですか（`Benzen_Perfect2025_v1/build_fresh/pico_copter.uf2` は **2026-08-28 に Pico 2 W 用へ移植済み**で Pico 1 には入りません。古い RP2040 版の UF2 か、`Benzen_Perfect2025_v1.zip` でしょうか） | 比較対象のコードが特定できないと A/B の因果が確定しない |
| **Q2** | `ESC_calib = 1` はどのファイルで変えましたか。Perfect2025 側では `pico_copter.cpp` も直しましたか | §1 の仮説の直接検証 |
| **Q3** | 使用した Pico 2 は **W** ですか、非 W ですか | 両 CMakeLists は `pico2_w`。非 W だと `cyw43_arch_init()` で止まる |
| **Q4** | GP2 / GP14 / GP15 / GP20 の 4 本は**全て**正常に回っていましたか | 「左側 2 本」なのか「chB 2 本」なのかで原因が変わる |
| **Q5** | LED は**何色と何色**が交互でしたか。そのときスイッチ（REDCIRCLE / LOG / HOVERING）はどの位置でしたか | §4 の 5 分岐のどれかが一意に決まる |
| **Q6** | 前回の「GP26」は GP20（Pico 物理ピン 26）でよいですか。今回は GP21 ですが、症状が変わったのか別の機体でしょうか | 症状の同一性の確認 |

---

## 付録: 今回確認した事実の出所

| 事実 | 出所 |
|---|---|
| `ESC_calib` の上書き | `Benzen_new/pico_copter.cpp:55`、`pwm.cpp:7`、ELF 内の `ESC_calib` シンボル |
| ビルドが最新であること | `pwm.cpp` 15:51 < `build_fresh/pico_copter.uf2` 15:58（同日） |
| SYS_CLK が 125 MHz | 実ビルドフラグでの `_Static_assert`、SDK `hardware/clocks.h:189-207` |
| `pwm.cpp` が実質同一 | `Benzen_new` と `Benzen_Perfect2025_v1` の diff（差分はサーボのみ） |
| IRQ 内 printf | `Benzen_new/control.cpp` の `rate_control()`、ELF 内 `"REDCIRCLE"` 文字列（Perfect2025 には無い） |
| `Flight_mode` 条件の差 | `Benzen_new/control.cpp:910,925,939` ⇔ `Benzen_Perfect2025_v1/control.cpp:866` |
| LED 分岐 | `Benzen_new/control.cpp` `led_control()` |
| ミキサ係数 | `Benzen_new/control.cpp` の有効なミキサ行 |
