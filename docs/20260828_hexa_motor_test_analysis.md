# ヘキサコプタ動作試験（2026-08-28）モータ回転異常の原因分析

- **供試体**: `drone_board_hexa_74HC245`（Raspberry Pi Pico 2 W / RP2350A2）
- **ESC**: GOKU HEX 13A（6-in-1）
- **モータ**: T-Motor F1404 KV2900
- **バッファ**: SN74LVC245APWR（RP2350-E9 対策として Pico GPIO ↔ ESC 間に挿入）
- **対象ファーム**: `Benzen_NMPC` / `Benzen_new`

---

## 0. 結論（先に要約）

観測された 2 つの症状は**別原因**であり、切り分けは正しい。

| # | 症状 | 原因 | 確度 |
|---|---|---|---|
| **A** | `Benzen_NMPC` でスロットルを動かしてもモータが一定回転のまま | **ソフト**。`#ifdef NMPC_BENCH_LOG` が `-DNMPC_BENCH_LOG=0` でも真になり、机上検証用（bench）の分岐が実飛行ビルドに混入。スロットル利得が 0.2 に落ち、**スティック 50 % 未満では全チャンネルが `motor_stop()`（=1108 µs 固定）**になる | **確定**（実 ELF のバイト列で確認） |
| **B** | 一部モータ（GP3 / "GP26"）だけ低スロットルで回らない | **ハード寄り**。PWM の出力下限が **1108 µs** で、ここが ESC の起動しきい値の真上にある。個体差でどのチャンネルが「じわ回り」するかが決まる。ESC スロットルキャリブレーション未実施が濃厚 | 高（要実測） |

**A のせいで機体は常時 1108 µs に張り付いていたため、B が「2 個だけ回らない」という形で顕在化した。**
`Benzen_new` ではスロットルが正しく 1108 → 1860 µs を掃引するので、しきい値を超えた時点で残り 2 個も回り始めた。観測と完全に一致する。

---

## 1. 症状の整理（報告内容）

| ファーム | 観測 |
|---|---|
| `Benzen_NMPC` | スロットルスティックをどう動かしても **GP3 / GP26 は回らず、他 4 個は一定回転のまま** |
| `Benzen_new` | スロットルに応じて **回転数が適切に変化**。ただし低スロットル域では GP3 / GP26 が回らず、ある値を超えると回り始める |

---

## 2. 原因 A: `#ifdef` と `-D...=0` の取り違え（ソフト・確定）

### 2-1. 何が起きているか

`Benzen_NMPC/CMakeLists.txt:75-79`

```cmake
target_compile_definitions(pico_copter PRIVATE
  SYS_CLK_KHZ=125000
  DEBUG_SKIP_LOCK_COM=0
  NMPC_BENCH_LOG=0      # ← "0" のつもりで書いているが「定義されている」
  NMPC_TR1_VERIFY=0     # ← 同上
  EIGEN_NO_MALLOC
)
```

C の前処理では **`#ifdef X` は「X の値」ではなく「X が定義されているか」しか見ない**。
`-DNMPC_BENCH_LOG=0` は「値 0 で定義済み」なので `#ifdef NMPC_BENCH_LOG` は **真**になる。

同ファイル内で `DEBUG_SKIP_LOCK_COM` だけは `#if DEBUG_SKIP_LOCK_COM`（`control.cpp:582`）と正しく書かれており、
**この 2 つだけが `#ifdef` になっている**。書き分けの事故である。

| 箇所 | 記法 | 判定 | 意図 |
|---|---|---|---|
| `control.cpp:582` | `#if DEBUG_SKIP_LOCK_COM` | 偽（正） | 正しい |
| `control.cpp:1092` | `#if defined(USE_NMPC) && defined(NMPC_BENCH_LOG)` | **真（誤）** | 偽であるべき |
| `control.cpp:1543` | `#ifdef NMPC_BENCH_LOG` | **真（誤）** | 偽であるべき |
| `control.cpp:206, 1636` / `nmpc.cpp:99, 533` | `#ifdef NMPC_BENCH_LOG` | **真（誤）** | bench 計装が常時コンパイル |
| `nmpc_params.hpp:99` | `#ifdef NMPC_TR1_VERIFY` | **真（誤）** | 偽であるべき |

### 2-2. 実バイナリでの確認（推測ではない）

`Benzen_NMPC/control.cpp:1092`

```cpp
#if defined(USE_NMPC) && defined(NMPC_BENCH_LOG)
  T_ref = 0.2f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
#else
  T_ref = 0.7f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
#endif
```

`0.2f * BATTERY_VOLTAGE`（= `0.2f * 7.4`）は double に昇格して
`1.4800000220537186` という**他では出てこない定数**になる。
本日ビルドの `build_fresh/pico_copter.elf`（2026-08-28 12:23）を走査した結果:

| 探索した定数 | ELF 内出現数 |
|---|---|
| `0.2f*7.4 = 1.4800000220537186`（bench 分岐） | **1** |
| `0.7f*7.4 = 5.179999911785126`（正規分岐） | **0** |

→ **実際に書き込まれたファームは bench 分岐でビルドされている。**
（比較のため `Benzen_new` の ELF には `0.5*7.4 = 3.7` と `0.7*7.4 = 5.18` が存在する。）

### 2-3. 波及 1: スロットル利得が 2/7 に低下 + 半分までデッドゾーン

`rate_control()` の停止判定（`control.cpp:1343`）は

```cpp
if (T_ref / BATTERY_VOLTAGE < Disable_duty)   // Disable_duty = 0.1
    motor_stop();
```

`T_ref = 0.2·V·r`（r はスティック比 0–1）なので `T_ref / V = 0.2·r`。
**`0.2·r < 0.1` すなわち `r < 0.5` で全モータ停止。**

`r ≥ 0.5` になると `u_hover = T_ref · T_ref_coe = 0.2 · 7.4 · r · 0.7 = 1.036·r` が一気に **0.518** から始まる。

### 2-4. スロットル → PWM パルス幅（実測可能な予測値）

PWM 設定は両ファーム共通で `wrap=3124, clkdiv=100, SYS_CLK=125 MHz`
→ **400 Hz / 周期 2.5 ms / 1 カウント = 0.8 µs**。
`set_duty_*()` は `duty` を `[0.01, 0.97]` にクランプしてから
`level = 1000·duty + 1375` を書くので、**出力パルスは 1108–1876 µs**。

| スティック比 r | `Benzen_new` duty | パルス [µs] | `Benzen_NMPC` duty | パルス [µs] |
|---|---|---|---|---|
| 0.00 | 0（停止） | **1108** | 0（停止） | **1108** |
| 0.10 | 0.222 | 1278 | 0（停止） | **1108** |
| 0.20 | 0.444 | 1455 | 0（停止） | **1108** |
| 0.30 | 0.666 | 1633 | 0（停止） | **1108** |
| 0.40 | 0.888 | 1810 | 0（停止） | **1108** |
| 0.428 | 0.950（飽和） | 1860 | 0（停止） | **1108** |
| 0.50 | 0.950 | 1860 | **0.518** | **1514**（不連続ジャンプ） |
| 0.60 | 0.950 | 1860 | 0.622 | 1597 |
| 0.92–1.0 | 0.950 | 1860 | 0.950 | 1860 |

> **`Benzen_NMPC` はスティック下半分で 1108 µs に完全固定** → 「いくら動かしても一定回転」。
> **`Benzen_new` は r=0.046 から連続的に 1278→1860 µs を掃引** → 「回転数が適切に変化」。
>
> 報告された症状と過不足なく一致する。

なお `r ≥ 0.5` を超えた瞬間に duty 0 → 0.518（1108 → 1514 µs）へ跳ぶ。
**プロペラを付けた状態でこのビルドのスロットルを半分以上入れるのは危険**なので、修正前に試さないこと。

### 2-5. 波及 2: NMPC がスロットルと無関係に走る

`control.cpp:1543-1547`

```cpp
#ifdef NMPC_BENCH_LOG
    nmpc_run = (Arm_flag == 2);                       // ← 机上検証用。常時実行
#else
    nmpc_run = (T_ref / BATTERY_VOLTAGE >= Flight_duty);
#endif
```

100 Hz の `angle_control()` は Arm 済みなら常に NMPC を回すのに、
400 Hz の `rate_control()` は `r < 0.5` で `motor_stop()` する。
**制御器と出力段の起動条件が食い違った状態**で飛ばしていたことになる。

### 2-6. 波及 3: Hessian の定期フル再構築が無効化されている

`nmpc_params.hpp:99-103`

```cpp
#ifdef NMPC_TR1_VERIFY
    constexpr int reset_period = 100000;   // ← こちらが選ばれている
#else
    constexpr int reset_period = 100;
#endif
```

TR1/SR2 のランク更新による数値誤差を洗い流す定期リセットが事実上無効
（100 周期 = 1 秒 → 100000 周期 = 1000 秒）。
今回の症状の直接原因ではないが、**長時間飛行で解が劣化する潜在バグ**。飛行前に直すべき。

### 2-7. 判定式そのものの不整合（`Benzen_new` では修正済み・NMPC 側は未反映）

`Disable_duty = 0.1` / `Flight_duty = 0.05` は **duty の閾値**である。
比較対象は「姿勢ゼロ時の共通 duty」= `T_ref * T_ref_coe` でなければならない。

| ファーム | 判定式 | 適否 |
|---|---|---|
| `Benzen_new:353, 1187, 1321` | `T_ref * T_ref_coe < Disable_duty` | **正**（修正済み） |
| `Benzen_NMPC:1288, 1343` | `T_ref / BATTERY_VOLTAGE < Disable_duty` | 誤（旧式のまま） |
| `Benzen_NMPC:1469, 1547` | `T_ref / BATTERY_VOLTAGE < Flight_duty` | 誤（旧式のまま） |

`Benzen_NMPC` の `u_hover` は `T_ref * T_ref_coe`（`control.cpp:1591`）で作られているのに、
起動判定だけ別スケールを見ている。**`Benzen_new` に入れた修正が NMPC 側へ横展開されていない。**

### 2-8. 定数の食い違い

| | `Benzen_new` | `Benzen_NMPC` |
|---|---|---|
| スロットル利得 | `0.5` | `0.2`（bench）/ `0.7`（正規） |
| `T_ref_coe` | **0.6** | **0.7** |
| 共通 duty = `T_ref·T_ref_coe` | `2.22·r` | `1.036·r`（bench）/ `3.626·r`（正規） |
| ホバー duty 0.129 に相当する r | **0.058** | 0.036（正規時） |

**現に飛んでいるのは `Benzen_new` の 0.5 / 0.6 の組み合わせ**なので、
NMPC 側もここへ合わせないと、bench 定義を直しただけでは今度は**スロットルが効きすぎる**。

---

## 3. 原因 B: 特定チャンネルだけ低スロットルで回らない（ハード寄り）

両ファームで再現しているので、**ソフトの分岐ではなく出力レンジと ESC 側の問題**。

### 3-1. `motor_stop()` は「真のゼロ」を出していない

```cpp
void set_duty_xx(float duty){
    if (duty>0.97) duty=0.97;
    if (duty<0.01) duty=0.01;          // ← 0 を渡しても 0.01 に持ち上げられる
    duty = (DUTYMAX-DUTYMIN)*duty + DUTYMIN;   // DUTYMIN=1375, DUTYMAX=2375
    ...
}
```

`motor_stop()` は `set_duty_*(0.0)` を呼ぶが、実際に出るのは **1385 カウント = 1108 µs**。
DUTYMIN 自体も 1375 カウント = **1100 µs**。

GOKU HEX 13A がスロットルキャリブレーション未実施（BLHeli_S 系の既定 ≈ 1040–1960 µs）だとすると、

- 1108 µs → 指令 **約 7 %**（＝停止指令ではない。じわ回りする）
- 1876 µs → 指令 **約 91 %**（上端も 9 % 損している）

**「モータを止めているつもりで 7 % を出し続けている」**状態であり、
F1404 KV2900 + プロペラの起動しきい値がちょうどこの付近にある。
だから **6 個中 4 個は回り、2 個は回らない**という個体差が出る。

### 3-2. ESC スロットルキャリブレーションを実施したか

`pwm.cpp` には `ESC_calib=1` でキャリブレーションを行う経路がある（電源投入時に DUTYMAX を 3 s → DUTYMIN を 1 s）。
現状は `ESC_calib=0`（`pwm.cpp:7`）。**新しい ESC に対して一度も実施していないなら、これが B の第一容疑者。**

> ⚠ `ESC_calib=1` は起動直後に全 6 ch へ **DUTYMAX（1900 µs）を 3 秒間**出す。
> **必ずプロペラを外して実施すること。**
> また DUTYMIN 保持が 1 秒しかないので、ESC が学習し切らない可能性がある（2 秒以上が無難）。

キャリブレーション後は ESC のレンジが 1100–1900 µs に一致し、
`duty=0`（1108 µs）がほぼ真の停止点になるので、B は原理的に解消する。

### 3-3. 74LVC245 の `/OE`（19 番ピン）が 10 kΩ で **3.3 V 側**に引かれている（要実測）

回路図・基板データから抽出したネットは以下の通り。

```
U4 pin19 (/OE, KiCad 記号上の名前は "CE")  ──┬── R3 10kΩ ── +3V3
                                            └── （他に接続なし）
U4 pin1  (DIR, "A->B")                     ─── +3V3        → A→B 方向。これは正しい
```

**74xx245 の 19 番ピンは Low アクティブの出力イネーブル**である。
データ上は 3.3 V に固定＝**出力 Hi-Z（常時ディセーブル）**になってしまう。

実機ではモータが回っている以上、実際の `/OE` は Low になっているはず。考えられるのは

1. R3 が未実装で `/OE` が**フローティング**（LVC はバスホールドを持たないため電位不定）
2. 基板改造（ジャンパ等）で GND に落ちている
3. 図面上の誤り（プルダウンのつもりで +3V3 に描いた）

**1. だとするとチャンネル間の挙動差・温度や電源変動での不安定さの原因になり得る**ため、
B の第二容疑者として **U4 の 19 番ピン電圧を実測**すること（Low = 0 V なら健全）。

### 3-4. RP2350-E9 対策自体は正しく書けている

`pwm_init()` は `gpio_set_function(..., GPIO_FUNC_PWM)` を先に全ピンへ掛け、
**その後で** `pwm_pins_disable_input()`（`gpio_set_input_enabled(pin,false)`）を呼んでいる。
`gpio_set_function()` は PADS の IE を立て直すので、この順序は必須であり **両ファームとも正しい**。
E9 は今回の症状には関与していないと判断する。

---

## 4. 配線対応表（実データから抽出）

### 4-1. GPIO → 74LVC245 → ESC コネクタ

| Pico GPIO | Pico 物理ピン | 245 A側 | 245 B側 | 基板コネクタ（リファレンス） | 回路図 Value | 基板 Value |
|---|---|---|---|---|---|---|
| GP2 | 4 | pin2 (A0) | pin18 (B0) | `GP2` | ESC1 | ESC6 |
| GP3 | 5 | pin3 (A1) | pin17 (B1) | `GP3` | ESC2 | ESC5 |
| **GP14** | 19 | pin4 (A2) | pin16 (B2) | **`GP4`** ← シルク誤記 | ESC3 | ESC4 |
| **GP15** | 20 | pin5 (A3) | pin15 (B3) | **`GP5`** ← シルク誤記 | ESC4 | ESC3 |
| GP20 | **26** | pin6 (A4) | pin14 (B4) | `GP20` | ESC5 | ESC2 |
| GP21 | 27 | pin7 (A5) | pin13 (B5) | `GP21` | ESC6 | ESC1 |
| GP6 (サーボ) | 9 | — | — | `GP6` (SRVO) | — | 245 を経由しない |

> ⚠ **コネクタ `GP4` / `GP5` は実際には GP14 / GP15 に繋がっている。** リファレンス名が実体と一致していない。
> ⚠ **ESC1..ESC6 の番号が回路図と基板データで逆順**（回路図 GP2=ESC1、基板 GP2=ESC6）。
> どちらかが古い。**モータ番号の議論では `GPxx` のリファレンス名を使い、ESC 番号は使わないこと。**

### 4-2. GPIO → PWM スライス → ファーム上のモータ

`pwm.cpp` の実際の書き込み先（コメントは両ファームで食い違っているが、**コードは完全に同一**）:

| 関数 | スライス / ch | GPIO | コネクタ |
|---|---|---|---|
| `set_duty_fr()` | slice7 A | GP14 | `GP4` |
| `set_duty_rr()` | slice7 B | GP15 | `GP5` |
| `set_duty_fl()` | slice2 A | GP20 | `GP20` |
| `set_duty_rl()` | slice2 B | GP21 | `GP21` |
| `set_duty_mr()` | slice1 A | GP2 | `GP2` |
| `set_duty_ml()` | slice1 B | GP3 | `GP3` |

**`pwm.cpp` は `Benzen_NMPC` と `Benzen_new` でスライス番号・チャンネル割り当てが 1 バイトも違わない**
（差はコメント文言だけ）。**したがって A の原因が `pwm.cpp` にないことは確定している。**

---

## 5. 修正案

### 5-1. 【必須】bench 定義の誤爆を止める

`Benzen_NMPC/control.cpp`

```diff
-#if defined(USE_NMPC) && defined(NMPC_BENCH_LOG)     // :1092
+#if defined(USE_NMPC) && NMPC_BENCH_LOG
```

```diff
-#ifdef NMPC_BENCH_LOG                                 // :206, :1543, :1636
+#if NMPC_BENCH_LOG
```

`Benzen_NMPC/nmpc.cpp`

```diff
-#ifdef NMPC_BENCH_LOG                                 // :99, :533
+#if NMPC_BENCH_LOG
```

`Benzen_NMPC/nmpc_params.hpp`

```diff
-#ifdef NMPC_TR1_VERIFY                                // :99
+#if NMPC_TR1_VERIFY
```

> `#if 未定義マクロ` は 0 と評価されるため、SIL など `-D` を渡さないビルドでも正しく偽になる。

### 5-2. 【必須】スロットル定数を `Benzen_new` に揃える

現に飛んでいる値に合わせる。

```diff
-  T_ref = 0.7f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
+  T_ref = 0.5f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
```

```diff
-float T_ref_coe = 0.7;    // control.cpp:129
+float T_ref_coe = 0.6;
```

### 5-3. 【必須】起動判定を duty スケールに統一（`Benzen_new` と同形）

`control.cpp:1288, 1343`

```diff
-  if (T_ref / BATTERY_VOLTAGE < Disable_duty)
+  const float common_duty = T_ref * T_ref_coe;   // 姿勢ゼロ時の PWM
+  if (common_duty < Disable_duty)
```

`control.cpp:1469, 1547`

```diff
-  if (T_ref / BATTERY_VOLTAGE < Flight_duty)
+  if (T_ref * T_ref_coe < Flight_duty)
```

```diff
-  nmpc_run = (T_ref / BATTERY_VOLTAGE >= Flight_duty);
+  nmpc_run = (T_ref * T_ref_coe >= Flight_duty);
```

### 5-4. 【B 対策】ESC・出力レンジ側

1. **プロペラを外して** `ESC_calib=1` でスロットルキャリブレーションを 1 回実施する
   （`pwm.cpp` の DUTYMIN 保持を 1 s → 2 s へ延ばすことを推奨）。
2. `U4` 19 番ピン（`/OE`）の電圧を実測。**High なら基板側の要修正**。
3. キャリブレーション後も残るようなら、`set_duty_*()` の `duty<0.01 → 0.01` クランプを見直し、
   `motor_stop()` だけは DUTYMIN 未満（真のゼロ）を出せるようにする。

---

## 6. 検証手順（プロペラを外して実施）

1. **修正前の再現**: `Benzen_NMPC` を書き込み、ESC 信号線（コネクタ `GP2`〜`GP21` の SIG）をオシロで観測。
   スティック下半分で **1108 µs 固定**、r≈0.5 で **1514 µs へ跳ぶ**ことを確認する。
   → 一致すれば原因 A は確定。
2. 5-1〜5-3 を適用して再ビルド。ELF に `1.4800000220537186` が**含まれないこと**を確認:
   ```bash
   python3 -c "import struct;d=open('build/pico_copter.elf','rb').read();print(d.count(struct.pack('<d',0.2*7.4)))"
   ```
   （0 になれば OK）
3. スティックを 0→100 % 掃引し、パルス幅が `Benzen_new` と同じ **1108 → 1860 µs** を連続に掃くことを確認。
4. `ESC_calib=1` でキャリブレーション → 全 6 個が同じスティック位置で回り始めることを確認。
5. `U4` pin19 の電圧を測定（0 V であること）。

---

## 7. 未確認事項 / 要確認

| # | 項目 | 状態 |
|---|---|---|
| Q1 | **「GP26」がどのピンを指すか** | 基板・ファームともに **GP26 は存在しない**。Pico の**物理ピン 26 = GP20**（コネクタ `GP20`）と読むのが最有力。回路図のネット名も `Net-(U1-Pad26)` = GP20。**GP2 の誤記の可能性もある** |
| Q2 | ESC スロットルキャリブレーションの実施有無 | 未確認 |
| Q3 | `U4` の `/OE` 実電圧 | 未実測（データ上は 3.3 V プルアップ＝ディセーブル） |
| Q4 | GOKU HEX 13A の入力プロトコル設定（PWM / OneShot / DShot） | 未確認。DShot 前提の設定なら 400 Hz PWM は別の挙動になる |
| Q5 | `Benzen_NMPC` の `T_ref_coe = 0.7` は意図的か | `Benzen_new` は 0.6。どちらが正か要判断 |

---

## 付録: 数値の出所

| 値 | 出所 |
|---|---|
| PWM 400 Hz / 2.5 ms / 0.8 µs per count | `pwm.cpp` `wrap=3124`, `clkdiv=100`, `CMakeLists.txt` `SYS_CLK_KHZ=125000` |
| パルス 1108–1876 µs | `pwm.hpp` `DUTYMIN 1375` / `DUTYMAX 2375`、`set_duty_*()` のクランプ `[0.01, 0.97]` |
| `BATTERY_VOLTAGE 7.4` | `control.hpp:23`（両ファーム共通） |
| `CH3MIN 368 / CH3MAX 1638` | `radio.hpp:16-17`（両ファーム共通） |
| `Disable_duty 0.1 / Flight_duty 0.05` | `control.cpp:170-171`（NMPC）, `:149-150`（new） |
| `U_duty_max 0.95` | `nmpc_params.hpp` `mpc::U_duty_max` |
| bench 分岐がビルドに入っている証拠 | `build_fresh/pico_copter.elf`（2026-08-28 12:23）に double `1.4800000220537186` が 1 個、`5.179999911785126` が 0 個 |
| コンパイル時の実定義 | `build_fresh/.../flags.make` に `-DNMPC_BENCH_LOG=0 -DNMPC_TR1_VERIFY=0` |
| 配線 | `drone_board_hexa_74HC245/drone_board.kicad_pcb` のネット・パッド情報 |
