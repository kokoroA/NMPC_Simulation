# 実装計画書: 原因 A（bench 分岐の誤爆によるスロットル不動作）の修正

- **作成日**: 2026-08-28
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/`
- **前提資料**: [`20260828_hexa_motor_test_analysis.md`](20260828_hexa_motor_test_analysis.md)
- **スコープ**: 原因 A（ソフト）のみ。**原因 B（ESC キャリブレーション / `/OE` 実測）は本計画に含めない。**

---

## 0. 目的と完了条件

### 目的

`Benzen_NMPC` のスロットル指令が `Benzen_new` と同一の特性でモータへ届くようにする。

### 完了条件（DoD）

1. ビルド成果物 ELF に `0.2f*BATTERY_VOLTAGE` 由来の定数が**存在しない**こと（機械的に検証）。
2. スロットル 0→100 % 掃引で ESC 信号が **1108 → 1860 µs を連続に**掃くこと（オシロ実測）。
3. スティック位置に対するパルス幅が `Benzen_new` と**一致**すること（同一スティック位置で ±10 µs 以内）。
4. `nmpc_run` がスロットル連動になっていること（低スロットルで NMPC が停止し、`iter_us` が 0 になる）。
5. `reset_period` が 100 に戻っていること。
6. 上記を decision に記録し、INDEX に追記していること。

---

## 1. 確定している事実（再掲・調査済み）

| 事実 | 確認方法 |
|---|---|
| `CMakeLists.txt` は `NMPC_BENCH_LOG=0` / `NMPC_TR1_VERIFY=0` を渡している | `build_fresh/.../flags.make` |
| 受け側が `#ifdef` のため **両方とも真** | ソース確認 |
| 実機 ELF に `0.2f*7.4 = 1.4800000220537186` が 1 個、`0.7f*7.4 = 5.179999911785126` が 0 個 | ELF バイト走査 |
| `pwm.cpp` は `Benzen_new` と**コメント以外バイト単位で同一** | 差分確認 |

---

## 2. 設計判断

### 判断 1: マクロを 2 つに割る（**推奨・採用**）

現状 `NMPC_BENCH_LOG` は **「計装」と「挙動の書き換え」の 2 つ**を同時に切り替えてしまっている。

| 用途 | 現在の gate | 内容 |
|---|---|---|
| 計装（ログ・テレメトリ） | `NMPC_BENCH_LOG` | `NmpcBenchSnapshot`, `debug_print_nmpc_bench()`, WiFi テレメトリ, `evaluate_cost()` |
| **挙動の書き換え** | `NMPC_BENCH_LOG` | `T_ref` 利得 0.2、`nmpc_run = (Arm_flag==2)` |

**これを一括で 0 にすると `iter_us` / `loop_us` のテレメトリまで消える。**
実機の計算時間実測（予稿の 8.7 ms / 10.6 ms の出所）は飛行試験でも取り続けたい。

→ **挙動側を `NMPC_BENCH_MODE` として分離する。**

| 新マクロ | 飛行ビルド | 机上ビルド | 効果 |
|---|---|---|---|
| `NMPC_BENCH_MODE` | **0** | 1 | `T_ref` 利得と `nmpc_run` 条件 |
| `NMPC_BENCH_LOG` | **1** | 1 | スナップショット / テレメトリ |
| `NMPC_BENCH_COST`（新設・任意） | **0** | 1 | `evaluate_cost()` の実行 |

> **却下した案**: 「`NMPC_BENCH_LOG` の定義自体を CMake から消す」。
> 値で ON/OFF したい意図が明確であり、受け側を直さないと同じ事故が再発する。
> また `#ifdef` のままだと「定義を消し忘れる」形で再発しうる。

> `NMPC_BENCH_COST` を分ける理由: `evaluate_cost()` は `Su`(85×102) との行列ベクトル積を含み、
> **概算 0.15–0.3 ms を毎周期消費している**（要実測）。
> 未解決課題 #1「制御周期全体を 10 ms 以内」に直接効く可能性があるため、飛行ビルドでは切れるようにする。
> `cost` は診断値であり制御には使われていない（`nmpc.cpp:534-539` で `cost_out` に入るだけ）。

### 判断 2: スロットル定数を `Benzen_new` に合わせる（**推奨・採用**）

`#if` に直しただけでは `T_ref = 0.7f·V·r`, `T_ref_coe = 0.7` になり、**今度は効きすぎる**。

| 案 | 利得 / `T_ref_coe` | ホバー(duty 0.129) | 飽和(duty 0.95) | 判定 |
|---|---|---|---|---|
| 現行（誤爆） | 0.2 / 0.7 | r=0.125 | r=0.917 | 下半分が死んでいる |
| **案 A（採用）** | **0.5 / 0.6** | **r=0.058** | **r=0.428** | `Benzen_new` と完全一致 |
| 案 B | 0.7 / 0.7 | r=0.036 | r=0.262 | ホバー窓が更に狭く、飽和が早すぎる |

**現に飛んでいるのは `Benzen_new` の 0.5 / 0.6 である。** 制御則（PID→NMPC）の比較実験をする以上、
スロットル特性は同一にしておかないと差が何に由来するか分からなくなる。よって案 A。

> ⚠ `T_ref_coe` は NMPC の `u_hover` を通じて **ADMM の box 制約 `Δu ∈ [-u_hover, U_duty_max - u_hover]`**
> にも効く。0.7 → 0.6 で同一スティック位置での `u_hover` が下がり、上側の余裕が増える。
> 姿勢制御の権限配分が変わるので、**変更後の初回は必ず係留 or 手持ちで確認する。**

### 判断 3: 起動判定を duty スケールへ統一（**採用**）

`Disable_duty` / `Flight_duty` は duty の閾値なので、比較対象は `T_ref * T_ref_coe`。
**`Benzen_new` には既にこの修正が入っているが `Benzen_NMPC` へ横展開されていない。**

---

## 3. 変更一覧

### 3-1. `CMakeLists.txt`

```diff
 target_compile_definitions(pico_copter PRIVATE
   SYS_CLK_KHZ=125000
   DEBUG_SKIP_LOCK_COM=0
-  NMPC_BENCH_LOG=0
-  NMPC_TR1_VERIFY=0
+  NMPC_BENCH_MODE=0   # 1 = 机上検証(スロットル利得0.2 / Arm_flag==2で常時NMPC)
+  NMPC_BENCH_LOG=1    # 1 = iter_us/loop_us 等のスナップショットとテレメトリ
+  NMPC_BENCH_COST=0   # 1 = evaluate_cost() を毎周期実行(周期時間を食う)
+  NMPC_TR1_VERIFY=0
   EIGEN_NO_MALLOC
 )
```

### 3-2. `control.cpp`

| 行 | 変更 |
|---|---|
| 129 | `float T_ref_coe = 0.7;` → `0.6;`（`Benzen_new` と一致） |
| 206 | `#ifdef NMPC_BENCH_LOG` → `#if NMPC_BENCH_LOG` |
| 396 | `#endif  // NMPC_BENCH_LOG` （コメントのみ、変更不要） |
| 1092–1097 | ↓ |
| 1288 | `T_ref / BATTERY_VOLTAGE < Disable_duty` → `T_ref * T_ref_coe < Disable_duty`（PID 経路・現ビルドでは非活性だが整合のため） |
| 1343 | 同上（**NMPC 経路・活性**） |
| 1469 | `T_ref / BATTERY_VOLTAGE < Flight_duty` → `T_ref * T_ref_coe < Flight_duty`（PID 経路） |
| 1543–1548 | ↓ |
| 1636 | `#ifdef NMPC_BENCH_LOG` → `#if NMPC_BENCH_LOG` |

**1092–1097:**

```diff
-#if defined(USE_NMPC) && defined(NMPC_BENCH_LOG)
+#if defined(USE_NMPC) && NMPC_BENCH_MODE
   // 机上検証でもスロットル CH3 に追従 (Arm_flag==2 時も同式)
   T_ref = 0.2f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
 #else
-    T_ref = 0.7f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
+    // Benzen_new と同一係数 (0.5 / T_ref_coe=0.6)。変えたら必ず実測し直すこと。
+    T_ref = 0.5f * BATTERY_VOLTAGE * (float)(Chdata[2] - CH3MIN) / (CH3MAX - CH3MIN);
 #endif
```

**1343（NMPC 経路の出力ゲート）:**

```diff
-    if (T_ref / BATTERY_VOLTAGE < Disable_duty)
+    const float common_duty = T_ref * T_ref_coe;  // 姿勢ゼロ時の PWM
+    if (common_duty < Disable_duty)
```

**1543–1548:**

```diff
-#ifdef NMPC_BENCH_LOG
+#if NMPC_BENCH_MODE
     // 机上検証: Arm_flag==2 (飛行準備完了) なら常に NMPC 実行
     nmpc_run = (Arm_flag == 2);
 #else
-    nmpc_run = (T_ref / BATTERY_VOLTAGE >= Flight_duty);
+    nmpc_run = (T_ref * T_ref_coe >= Flight_duty);
 #endif
```

### 3-3. `nmpc.cpp`

| 行 | 変更 |
|---|---|
| 99 | `#ifdef NMPC_BENCH_LOG` → `#if NMPC_BENCH_COST`（`evaluate_cost()` の定義） |
| 533 | `#ifdef NMPC_BENCH_LOG` → `#if NMPC_BENCH_COST`（呼び出し） |

> 定義と呼び出しを**同じマクロ**で括ること。片方だけ変えると未使用関数警告か未定義参照になる。

### 3-4. `nmpc_params.hpp`

```diff
-#ifdef NMPC_TR1_VERIFY
+#if NMPC_TR1_VERIFY
     constexpr int   reset_period = 100000;
 #else
     constexpr int   reset_period = 100;
 #endif
```

> `#if 未定義マクロ` は 0 と評価される。SIL など `-D` を渡さないビルドでも `reset_period = 100` になり壊れない。

---

## 4. 実装手順

**すべて 1 コミットにまとめず、下記の単位で分ける**（後から二分探索できるようにするため）。

| # | 作業 | 単独で検証可能か |
|---|---|---|
| 1 | `nmpc_params.hpp` の `#if` 化 | ○（`reset_period` を静的に確認） |
| 2 | `nmpc.cpp` の `NMPC_BENCH_COST` 化 + CMake にマクロ追加 | ○（ビルドが通る／`cost` が 0 になる） |
| 3 | `control.cpp` の `NMPC_BENCH_MODE` 分離 + CMake | ○（ELF から 1.48 が消える） |
| 4 | スロットル定数を 0.5 / 0.6 へ | ○（ELF に 3.7 が現れる） |
| 5 | 起動判定を `T_ref * T_ref_coe` へ統一（4 箇所） | ○（オシロで低スロットル側の閾値が r=0.045 になる） |

> ⚠ **手順 3 だけを適用して通電しないこと。** 3 だけだと `0.7f / T_ref_coe=0.7` になり、
> r=0.036 でホバー・r=0.262 で飽和という**極端に敏感な**特性になる。
> **3・4・5 は必ずまとめて適用してから通電する。**

### コミット単位

- `fix(nmpc): -D...=0 を #ifdef で受けていた誤爆を修正し bench 挙動を NMPC_BENCH_MODE へ分離`
- `fix(nmpc): スロットル係数と起動判定を Benzen_new に整合`

> `Benzen_NMPC` は `pico-sdk` サブモジュール配下。**コードはサブモジュール側でコミットし、
> decision は親リポジトリの `docs/ai/decisions/` に書く**（`skills.md` §2）。
> decision には**サブモジュール側のコミットハッシュを残す**こと。

---

## 5. ビルドと静的検証（通電前に必ず実施）

```bash
cd program/pico/pico-sdk2/pico-sdk/Benzen_NMPC
rm -rf build_verify && cmake -S . -B build_verify -DCMAKE_BUILD_TYPE=Release && cmake --build build_verify -j
```

### 5-1. `-D...=` を渡している全マクロの受け側を機械的に検査

```bash
grep -n "ifdef NMPC_\|defined(NMPC_\|ifdef DEBUG_SKIP" *.cpp *.hpp
```

`NMPC_WIFI_TELEM` は値なしの定義（`NMPC_WIFI_TELEM=1`）なので `#ifdef` のままでも動くが、
**`=0` を渡す可能性があるマクロは 1 つ残らず `#if` にすること。**

### 5-2. ELF に bench 定数が入っていないことを確認

```bash
python3 -c "import struct;d=open('build_verify/pico_copter.elf','rb').read();f=lambda x:struct.unpack('<f',struct.pack('<f',x))[0];print('bench(0.2f*7.4):',d.count(struct.pack('<d',f(0.2)*7.4)),' new(0.5f*7.4):',d.count(struct.pack('<d',f(0.5)*7.4)))"
```

**期待値: `bench: 0`, `new: 1`**

### 5-3. `reset_period` の確認

```bash
grep -n -A4 "NMPC_TR1_VERIFY" nmpc_params.hpp
```

---

## 6. 実機検証手順

### フェーズ 1: プロペラを外して（必須）

1. ESC 信号線（コネクタ `GP2` / `GP3` / `GP4` / `GP5` / `GP20` / `GP21` の SIG）をオシロで観測。
2. スロットル 0 → 100 % をゆっくり掃引し、パルス幅を記録。

| スティック比 r | 期待パルス [µs] |
|---|---|
| 0.00 – 0.045 | 1108（停止） |
| 0.05 | 1189 |
| 0.10 | 1278 |
| 0.20 | 1455 |
| 0.30 | 1633 |
| 0.40 | 1810 |
| 0.428 以上 | 1860（飽和） |

3. **同じ機体・同じ送信機で `Benzen_new` を書き込み、同一スティック位置で同一パルス幅になることを確認。**
   これが最も確実な回帰テストである。
4. テレメトリ（または `NMPC_BENCH_SERIAL_LOG=1` ビルド）で
   - 低スロットル時に `nmpc=0`（`nmpc_active`）、`iter_us=0`
   - `T_ref * 0.6 ≥ 0.05` を超えたところで `nmpc=1` に変わる
   ことを確認。

### フェーズ 2: `NMPC_BENCH_COST` の効果測定

`NMPC_BENCH_COST=0` と `=1` の 2 ビルドで `iter_us` の平均・最大を比較し、差分を記録する。
**未解決課題 #1（周期 10 ms 超過）に対する寄与を定量化する。**

### フェーズ 3: 通電飛行前

原因 B（ESC キャリブレーション、`U4` pin19 の `/OE` 実測）を先に片付けること。
**A を直しても B が残っていると、低スロットル域で 6 個中 2 個が回らない状態は解消しない。**

### フェーズ 4: 係留 / 手持ちでの姿勢確認

`T_ref_coe` を 0.7 → 0.6 に下げたことで `u_hover` が変わり、ADMM の box 制約の余裕が変化している。
**ロール／ピッチの応答が変わっていないかを、離陸前に必ず確認する。**

---

## 7. リスクと対策

| # | リスク | 対策 |
|---|---|---|
| R1 | 手順 3 のみ適用した状態で通電し、r=0.036 で浮いてしまう | **3・4・5 をまとめて適用**。段階検証は静的検査のみで行う |
| R2 | `NMPC_BENCH_LOG=1` を残したことで周期時間が延びる | `NMPC_BENCH_COST=0` で最も重い部分を切る。フェーズ 2 で定量化 |
| R3 | `NMPC_BENCH_LOG=0` にしてテレメトリを失う | 判断 1 のマクロ分離で回避済み |
| R4 | `T_ref_coe` 変更で姿勢制御の権限配分が変わる | フェーズ 4 で係留確認 |
| R5 | 修正後も原因 B が残り「直っていない」と誤判定する | オシロでパルス幅を見る（モータの回転で判定しない） |
| R6 | サブモジュールでコミットし、親リポジトリに記録が残らない | decision にサブモジュールのコミットハッシュを記載 |

### ロールバック

`Benzen_NMPC` はサブモジュール側の git で管理されている。問題が出たら該当コミットを revert し、
`build_fresh/pico_copter.uf2`（2026-08-28 12:23 ビルド、現状の挙動）へ戻せる。
**ただしこの uf2 は bench 誤爆版なので、比較用途以外で通電しないこと。**

---

## 8. 本計画に含めないもの

- **原因 B**（ESC スロットルキャリブレーション、`U4` の `/OE` プルアップ、`set_duty_*()` の
  `duty<0.01 → 0.01` クランプ）。別タスクとする。
- 基板コネクタ `GP4` / `GP5` のシルク誤記、ESC1..6 の番号逆転（図面修正）。
- SIL への `Benzen_NMPC` 対応（M6）。**実装済みなら本修正を通電前に検証できたはずであり、
  優先度を上げる根拠になる。**
- `Benzen_new` 側の変更。**今回は一切触らない**（比較の基準を動かさないため）。

---

## 9. 完了後に記録すること（`skills.md` §2）

- `docs/ai/decisions/20260828_nmpc_bench_define_throttle.md` に**検証結果**を追記
  （現在は「未検証」で止めてある。オシロ実測値・`iter_us` の前後比較を入れる）。
- `NMPC_BENCH_COST` の効果が有意なら、`docs/ai/repository_rules.md` §5「現在の到達点」の
  **「ソルバ本体 平均 8.7 ms / 周期全体 10.6 ms」を更新**する（§5 は追記でなく上書き）。
- 昇格済み: `docs/ai/global_drafts.md` の「`-DX=0` を `#ifdef X` で受けてはならない」。
