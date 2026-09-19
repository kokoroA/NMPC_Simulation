# 実装計画書: 耐故障制御（FTC）モード — FR モータ停止と NMPC への通知

- **作成日**: 2026-09-01
- **対象**: `Benzen_NMPC/`（`radio.hpp`, `control.cpp`, `control.hpp`, `nmpc.cpp`, `nmpc.hpp`, `nmpc_params.hpp`）
- **要求**: プロポの `#define FTC 5` チャンネルで耐故障モードを切替。ON で **FR モータを停止**し、
  **NMPC 側にも FR が使えないことを認識させる**。LED は**通常状態と全周青の交互点滅**。
- **前提資料**: [`repository_rules.md §3`](ai/repository_rules.md)、[`20260831_nmpc_v4_n14_analysis.md`](20260831_nmpc_v4_n14_analysis.md)
- **状態**: **2026-09-01 実装完了・ビルド成功**（§9 に実施記録）

## 決定事項（ユーザ回答）

| # | 決定 |
|---|---|
| Q1 | **REDCIRCLE の処理はすべて削除。** FTC スイッチ ON で `Flight_mode = MODE_FTC` に入り、対象モータを止め、η を box 制約と `U_ref` シフトに反映。OFF で 6 発の η に戻し、モータを回し直す |
| Q2 | **スイッチで ON / OFF 両方向**に切り替える（ラッチしない） |
| Q3 | **REDCIRCLE モードは削除** |
| Q4 | **η は一気に 0**（ランプさせない） |
| Q5 | **テレメトリに `ftc` 列を追加**する |
| — | **停止させるモータは 1 行で変更できるようにする** → `Ftc_target_motor` |

**追加要求**: 受信機フェイルセーフと脱出条件も本計画に含める（§8）。

---

## 0. ⚠ 先に直す必要があるもの — 現在ビルドが通りません

`radio.hpp` の `#define REDCIRCLE 5` が `#define FTC 5` に置き換わった結果、
`control.cpp` の 4 箇所（469, 1015, 2070, 2568 行）で `REDCIRCLE` が未定義になり、**コンパイルエラーです。**

```
control.cpp:469:44: error: 'REDCIRCLE' was not declared in this scope
```

さらに構造上の問題があります。

> **`REDCIRCLE` は「チャンネル番号」と「Flight_mode の値」の両方に使われていました。**
> `Chdata[REDCIRCLE]`（＝ch5 を読む）と `Flight_mode == REDCIRCLE`（＝Flight_mode を 5 と比較）が同居しており、
> `control.hpp` の `#define REDCIRCLE 3` はコメントアウトされているため、**たまたま両方が 5 で動いていました。**
> `HOVERING`（＝8）も同じ構造です。

**したがって ch5 を FTC に転用するには、`Flight_mode` の選択ロジックから ch5 を外す必要があります。**
外さないと、**FTC スイッチを倒した瞬間に Flight_mode も変わります。**

---

## 1. 設計方針

`repository_rules.md §3` に従います。

> 耐故障は `η` を **box 制約の縮減と `U_ref` シフト**で表現する。
> 故障パターンごとの分岐や専用ミキシングは書かず、ソルバに再配分を最適化させる。

つまり **FR 専用のミキシングは書きません。** モータごとの健全度 `η ∈ [0,1]` を導入し、
NMPC の box 制約を絞るだけにします。ソルバが残り 5 発への再配分を自分で最適化します。

### 1-1. `η` の導入

```cpp
// 0.0 = 完全故障 / 1.0 = 健全。順序は [FR, FL, MR, ML, RR, RL]
float g_motor_eta[6] = {1,1,1,1,1,1};
```

FTC モード ON で `g_motor_eta[0] = 0.0f`（FR）。将来ほかの故障パターンを試すときも
この配列を書き換えるだけで済みます。

### 1-2. box 制約の縮減（NMPC 側の「認識」の実体）

現在の ADMM 射影はスカラー 1 組です。

```cpp
// nmpc.cpp:325  現状
inline void delta_u_bounds(float u_hover, float &du_min, float &du_max)
{
    du_min = -u_hover;
    du_max = mpc::U_duty_max - u_hover;
}
```

これを**モータごと**にします。

```
du_min[j] = -u_hover
du_max[j] = η[j] * U_duty_max - u_hover
```

- `η=1` → 従来どおり `[-u_hover, 0.95-u_hover]`
- **`η=0` → `du_min = du_max = -u_hover`** → **Δu が −u_hover に固定 → duty = 0**

**これが「NMPC に FR が回らないことを認識させる」の実体です。**
ソルバは FR に頼れないことを最適化の中で知り、残り 5 発で姿勢を作ります。

> **Hessian の再構築は不要です。** `H = Suᵀ Q̄ Su + R̄` と LLT 分解は box 制約に依存せず、
> 変わるのは ADMM の射影だけです。**34 ms の停止は発生しません。**

### 1-3. `U_ref` シフト

`U_ref = 0`（Δu の参照）のままだと、FR に強制される `Δu = -u_hover` に対して
`R_u` が定常的にペナルティを課し続け、ADMM 2 反復では収束が鈍ります。

```
U_ref[FR]    = -u_hover                      （強制値そのもの → ペナルティ 0）
U_ref[健全]  = +u_hover / n_healthy          （失った推力を残りで肩代わりする分）
```

`n_healthy = 5`。**これが `repository_rules` の言う「`U_ref` シフト」です。**

### 1-4. 出力側の二重ガード

`rate_control()` は `minimum_duty = Disable_duty = 0.1` で下から丸めます。
**このままだと NMPC が 0 を出しても FR は 0.1（≒1180 µs）で回り続けます。**

→ **`angle_control`（NMPC）と `rate_control`（出力）の両方**で `η=0` のモータを 0 にします。
片方を通り抜けても止まるようにするためです。

---

## 2. 変更一覧

### 2-1. `radio.hpp` — 変更なし

`#define FTC 5` はそのまま使います。

### 2-2. `control.hpp` — Flight_mode の値を独立させる

```diff
 #define NORMAL 0
-// #define ROCKING 1
-// #define AUTO 2
-// #define REDCIRCLE 3
+#define MODE_HOVERING 1
+// ch 番号（radio.hpp の HOVERING=8 等）と Flight_mode の値を混ぜないこと
```

> `Flight_mode == HOVERING`（＝8）も同じ混同です。**今回まとめて分離します。**

### 2-3. `control.cpp` — Flight_mode 選択から ch5 を外す

`rate_control()` 1013–1050 行付近。

```diff
-  if ((Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[HOVERING] < 200) && i2c_connect == 1)
+  if ((Chdata[LOG] < 200) && (Chdata[HOVERING] < 200))
   {
     Flight_mode = NORMAL;
     ...
   }
-  else if ((Chdata[REDCIRCLE] > 500) && (Chdata[LOG] < 200) && (Chdata[HOVERING] > 500) && i2c_connect == 1)
+  else if ((Chdata[LOG] < 200) && (Chdata[HOVERING] > 500))
   {
-    Flight_mode = HOVERING;
+    Flight_mode = MODE_HOVERING;
   }
-  else if ((Chdata[REDCIRCLE] < 200) && (Chdata[LOG] < 200) && (Chdata[HOVERING] < 200))
-  {
-    Flight_mode = REDCIRCLE;     // ← 削除（red_circle は UART 由来で未接続）
-  }
```

**REDCIRCLE モードは削除します**（`red_circle` は外部 UART から来る値で、現在配線されていません）。
2070 行・2568 行の `REDCIRCLE` 参照も同時に整理します。

### 2-4. `control.cpp` — FTC スイッチの読み取り

`rate_control()` 内、Flight_mode 判定の直後に置きます（400 Hz）。

```cpp
// ---- 耐故障(FTC)モード: ch5 で FR モータを故障扱いにする ----
// 200-500 は不感帯（前回値保持）。既存のモード判定と同じ流儀。
if (Arm_flag == 2) {
  if (Chdata[FTC] > 500)      g_ftc_mode = 1;
  else if (Chdata[FTC] < 200) g_ftc_mode = 0;
}
g_motor_eta[0] = (g_ftc_mode ? 0.0f : 1.0f);   // FR
```

### 2-5. `nmpc.hpp` / `nmpc.cpp` — box 制約をモータごとに

```diff
+// モータ健全度 η（0=故障, 1=健全）。順序 [FR,FL,MR,ML,RR,RL]
+void nmpc_set_motor_eta(NmpcContext &ctx, const float eta[mpc::NU]);
```

`NmpcContext` に `float motor_eta[mpc::NU];`（24 B）を追加。`nmpc_init` で全 1.0。

```diff
-inline void delta_u_bounds(float u_hover, float &du_min, float &du_max)
-{
-    du_min = -u_hover;
-    du_max = mpc::U_duty_max - u_hover;
-    if (du_max < du_min) du_max = du_min;
-}
+// モータごとの Δu 範囲を UStackVec に展開する（η による box 縮減）
+inline void delta_u_bounds_vec(const NmpcContext &ctx, float u_hover,
+                               UStackVec &du_min, UStackVec &du_max)
+{
+    for (int j = 0; j < mpc::NU; ++j) {
+        float lo = -u_hover;
+        float hi = ctx.motor_eta[j] * mpc::U_duty_max - u_hover;
+        if (hi < lo) hi = lo;                    // η=0 なら lo==hi → duty 0 に固定
+        for (int k = 0; k < mpc::N; ++k) {
+            du_min(k * mpc::NU + j) = lo;
+            du_max(k * mpc::NU + j) = hi;
+        }
+    }
+}
```

`admm_solve` の射影を差し替えます。

```diff
-    ctx.ws.z = (ctx.ws.U + ctx.ws.lambda).cwiseMin(du_max).cwiseMax(du_min);
+    ctx.ws.z = (ctx.ws.U + ctx.ws.lambda).cwiseMin(ctx.scratch.du_max).cwiseMax(ctx.scratch.du_min);
```

`Scratch` に `UStackVec du_min, du_max;` を追加（N=14 で 84 要素 ×2 = **672 B**）。
**`EIGEN_NO_MALLOC` のまま静的確保です。**

`nmpc_step` の末尾にある `U_opt` のクリップも同じ範囲を使うよう直します
（現在は `delta_u_bounds` のスカラーを再計算しています）。

### 2-6. `control.cpp` — `U_ref` シフトと出力ガード

`angle_control()` の NMPC 分岐。

```diff
       for (int k = 0; k < nmpc::mpc::N; ++k)
       {
         for (int i = 0; i < nmpc::mpc::NX; ++i)
           X_ref_stack(k * nmpc::mpc::NX + i) = x_ref(i);
-        // 加算合成: NMPC は Δu を最適化、参照は 0
-        for (int j = 0; j < nmpc::mpc::NU; ++j)
-          U_ref_stack(k * nmpc::mpc::NU + j) = 0.0f;
+        // 加算合成: 健全時は 0。故障時は η に応じてシフト（repository_rules §3）
+        for (int j = 0; j < nmpc::mpc::NU; ++j)
+          U_ref_stack(k * nmpc::mpc::NU + j) = u_ref_shift[j];
       }
```

`u_ref_shift` は毎周期 1 回だけ作ります。

```cpp
float u_ref_shift[6] = {0,0,0,0,0,0};
int n_healthy = 0;
for (int j = 0; j < 6; ++j) if (g_motor_eta[j] > 0.5f) n_healthy++;
if (n_healthy > 0 && n_healthy < 6) {
  for (int j = 0; j < 6; ++j)
    u_ref_shift[j] = (g_motor_eta[j] > 0.5f) ? (u_hover * (6 - n_healthy) / n_healthy)
                                             : (-u_hover);
}
```

duty 生成のあと、故障モータを 0 に落とします。

```diff
       RL_duty = mix_duty(Delta_opt(5));
+      // 故障モータは 0（box 制約でも 0 になるが、ここでも落として二重に保証する）
+      float *d6[6] = {&FR_duty,&FL_duty,&MR_duty,&ML_duty,&RR_duty,&RL_duty};
+      for (int j = 0; j < 6; ++j) if (g_motor_eta[j] <= 0.5f) *d6[j] = 0.0f;
```

`rate_control()` の clamp。

```diff
     for (int i = 0; i < 6; ++i)
     {
       if (*duties[i] != *duties[i]) *duties[i] = minimum_duty;
-      if (*duties[i] < minimum_duty) *duties[i] = minimum_duty;
+      if (g_motor_eta[i] <= 0.5f) { *duties[i] = 0.0f; continue; }   // 故障モータは下限を適用しない
+      if (*duties[i] < minimum_duty) *duties[i] = minimum_duty;
       if (*duties[i] > maximum_duty) *duties[i] = maximum_duty;
     }
```

> `set_duty_fr(0.0f)` は内部で `duty<0.01 → 0.01` に丸められ **1108 µs** を出します。
> ESC キャリブレーション済み（1100–1900 µs）なので、これは**最小＝停止**です。
> 停止しない場合は `pwm.cpp` 側で `DUTYMIN` を直接書く経路を用意します（§6 R3）。

### 2-7. `control.cpp` — LED

`led_control()` の `Arm_flag == 2` の連鎖の**先頭**に入れます（Flight_mode と直交するため）。

```cpp
else if (Arm_flag == 2 && g_ftc_mode)
{
  // 耐故障モード: 通常表示 ⇔ 全周青 の交互（4 Hz）
  if (cnt == 0)  rgbled_normal();
  if (cnt == 50) rgbled_blue();
  cnt++;
  if (cnt == 100) cnt = 0;
}
```

`rgbled_blue()` は既存です（`rgbled.cpp:199`）。周期は既存の点滅分岐と同じ 100 カウント（400 Hz → 4 Hz）。

### 2-8. テレメトリ（任意・推奨）

`NmpcTelPkt` に **未使用の `uint8_t _pad` があります**。これを `uint8_t ftc` に変えれば
**パケットサイズを変えずに** FTC 状態を送れます。CSV には `ftc` 列を 1 つ足します。

> 足さなくても **`u_fr` が 0.000 になる**ので、ログからは判別可能です。
> **Phase 1 では省略してよい**と考えます。

---

## 3. 実装手順

| # | 作業 | 単独で検証できるか |
|---|---|---|
| **1** | `REDCIRCLE` の後始末（§2-2, §2-3）。**ビルドを通す** | ○ ビルド成功 |
| 2 | `g_ftc_mode` / `g_motor_eta` の追加と ch5 読み取り（§2-4） | ○ テレメの `u_fr` で確認 |
| 3 | LED（§2-7） | ○ 目視 |
| 4 | NMPC の box 制約をモータごとに（§2-5） | ○ `η=全1` で従来と同一結果になること |
| 5 | `U_ref` シフトと出力ガード（§2-6） | ○ 係留で FR が止まること |

**1 → 2 → 3 で「FR が止まる・LED が変わる」ところまで先に作り、
4 → 5 で NMPC に認識させる**、という順序を推奨します。
3 まででも FR は止まりますが、**NMPC は FR がまだ使えると思ったまま**なので飛ばさないでください。

---

## 4. 検証

### 4-1. 静的（通電前）

```bash
# η が全 1 のとき、従来と同じ Δu 範囲になることを確認
static_assert(nmpc::mpc::NU == 6 && nmpc::mpc::N == 14);
```

- `η = {1,1,1,1,1,1}` でビルドし、**v4 と同じ挙動になること**（回帰）
- `bss` の増加が 700 B 程度に収まること（`UStackVec` ×2 ＋ `motor_eta`）
- `iter_us` が増えないこと（射影がスカラー→ベクタになるだけ）

### 4-2. 机上（プロポあり・プロペラ無し）

1. スロットルを上げて全 6 発が回ることを確認
2. **ch5 を ON** → **FR だけ止まり、他 5 発は回り続ける**
3. LED が通常⇔全周青の交互になる
4. ch5 を OFF → FR が復帰する
5. テレメで `u_fr = 0.000`、`u_fl..u_rl` が **ON 前より上がっている**こと
   （残り 5 発が肩代わりしている ＝ `U_ref` シフトが効いている）

### 4-3. 係留

1. 健全でホバー直前まで上げ、`u_base` を記録
2. ch5 を ON → **姿勢が保てるか**、`u_base` をどれだけ上げる必要があるか
3. `iter_us` / `loop_us` が変わらないこと（**フル再構築が走らないこと**）

### 4-4. 飛行

**健全飛行が安定してから。** 高度を十分に取り、ch5 を ON にして姿勢が保てるかを見ます。
**最初は必ず係留か、低高度かつ広い場所で。**

---

## 5. 期待される挙動と物理的な余裕

| | 健全 6 発 | **FTC 5 発** |
|---|---|---|
| 1 発あたり必要推力 | 0.523 N | **0.628 N**（×1.2） |
| 必要 duty（同定マップ） | 0.51 | **約 0.62** |
| 上限 0.95 までの余裕 | 0.44 | **0.33** |
| 推力マージン | 1.50 | **1.25** |

**姿勢制御に使える余裕が 0.44 → 0.33 に減ります。** 飽和しやすくなるので、
FTC 中は姿勢の追従が悪化する見込みです。**これは想定内で、研究上の測定対象そのものです。**

**ヨーについて**: `kS = [-1, +1, +1, -1, -1, +1]`（FR..RL）。FR を失うと反トルクの均衡が崩れ、
残り 5 発では `kS` の総和が 0 になりません。**ヨーに定常的な偏りが出ます。**
`Q_r = 20` でソルバが吸収しようとしますが、**ヨー権限は `C_QT ≈ 0.036` と弱い**ため、
**FTC 中はヨーが回り続ける可能性が高い**です。これも想定内。

---

## 6. リスクと対策

| # | リスク | 対策 |
|---|---|---|
| **R1** | ch5 が Flight_mode 選択にも使われており、FTC を倒すとモードも変わる | §2-3 で ch5 を Flight_mode から外す。**これを飛ばすと必ず踏む** |
| **R2** | 出力段の `minimum_duty = 0.1` により FR が止まらない | §2-6 で故障モータは下限をスキップ。**二重ガード** |
| R3 | `set_duty_fr(0.0)` が 1108 µs を出すため、ESC が完全停止しない可能性 | 机上で確認。止まらなければ `pwm.cpp` に `DUTYMIN` 直書きの停止経路を追加 |
| R4 | FTC ON の瞬間に姿勢が乱れる（推力が 1/6 消える） | 高度を取ってから。将来は `η` をランプさせる（一気に 0 にせず 0.5 s かけて落とす） |
| R5 | FTC OFF で FR が急に復帰し、逆向きの過渡が出る | 同上。**まず OFF に戻さない運用で試す** |
| R6 | `U_ref` シフトで健全 5 発の baseline が上がり、飽和が早まる | 係留で `u_base` の必要量を測ってから飛ばす |
| R7 | box が `du_min == du_max` になり ADMM が数値的に不安定になる | `η=0` でも `hi = lo` は正しい射影（1 点への射影）。**ただし机上で `iter_us` と姿勢の暴れを必ず確認** |

---

## 7. 本計画に含めないもの

- **FDI（故障検出）**。今回は**スイッチによる手動宣言**のみです。自動検出は別タスク
- 2 発以上の故障。`g_motor_eta` の配列を変えれば表現できますが、**推力マージン 1.25 では 2 発は成立しません**
- `η` の中間値（部分故障）。box は連続的に縮むので原理的には動きますが、未検証
- 受信機フェイルセーフ・脱出条件（[未実装のまま](20260831_nmpc_v4_n14_analysis.md)）。
  **FTC より先に入れることを強く勧めます** —— 意図的に故障を作る実験をするなら尚更です

---

## 8. フェイルセーフと脱出条件

意図的に故障を作る実験をする以上、**これが無い状態で飛ばすべきではありません。**
FTC と同じコミットで入れます。

### 8-1. 判定と原因コード

`Failsafe_flag` は**一度成立したらラッチ**し、`Arm_flag` が 2 でなくなる（＝再アーム）まで解除しません。
目的は「操縦不能になった機体をそれ以上飛ばさない」ことなので、自動復帰はさせません。

| 原因 | 条件 | 根拠 |
|---|---|---|
| `FS_RX_LOST` | `Chdata[16] & (0x08 \| 0x04)` | SBUS のフラグバイト（failsafe / frame lost）。**取得済みだが今まで一度も参照していなかった** |
| `FS_RX_TIMEOUT` | 最終フレームから **200 ms** 経過 | SBUS は 14 ms 周期。フラグが来ないまま途絶する場合を拾う |
| `FS_ATTITUDE` | \|φ\| または \|θ\| > **60°** | スティック指令 ±54° に対してマージンを取り、フルスティックで誤発火しない値 |
| `FS_RATE` | \|p\|, \|q\|, \|r\| > **300 deg/s** | 0831 の横転時に 500 deg/s 超を記録している |
| `FS_OVERG` | `acc_norm` > **150 m/s²** | 旧値 800（≈81.5 g）は `LSM9DS1_16g`（最大 157、3 軸合成 272）では**到達不能**だった |

### 8-2. 受信フレーム時刻の記録（`radio.cpp`）

途絶を時間で見るために、フレーム完結時刻を残します。

```c
// radio.cpp: chars_rxed==25 のところ
Radio_last_frame_us = time_us_32();
```

`Radio_last_frame_us == 0`（起動から 1 フレームも受けていない）では**判定しません**。
受信機が繋がっていない状態で誤って発火させないためです。

### 8-3. しきい値の決め方

⚠ **加速度 150 m/s²（≈15 g）は「明らかに衝突」の水準に置いています。**
誤検知で飛行中にモータが止まる方が危険なので、**高めから始めます。**
係留ログで `acc_norm` の分布を見てから下げてください。

姿勢 **60°** は、`update_attitude_refs_from_sticks()` のロール/ピッチ指令を **±54°** に揃えた前提で、
**フルスティックでも誤発火しない**ように 6° のマージンを取った値です。
指令スケールを広げる場合は、このしきい値も一緒に見直してください。

角速度 300 deg/s は、0831 の横転時に 500 deg/s 超を記録していることから、
**通常の操縦では到達しない**水準として置いています。

---

## 9. 実施記録（2026-09-01）

### 変更したファイル

| ファイル | 行 | 内容 |
|---|---|---|
| `control.hpp` | 40 | `MODE_HOVERING` / `MODE_FTC`、`MOTOR_*`、`Ftc_*` / `Motor_eta` / `Failsafe_*` の宣言 |
| `control.cpp` | 228 | グローバル、REDCIRCLE 削除、FTC スイッチ、LED、`U_ref` シフト、出力ガード、`failsafe_check()` |
| `nmpc.hpp` | 12 | `motor_eta[]`、`du_min`/`du_max`、`nmpc_set_motor_eta()` |
| `nmpc.cpp` | 55 | `delta_u_bounds_eta()` / `delta_u_bounds_fill()`、ADMM 射影のベクタ化 |
| `radio.hpp` / `radio.cpp` | 5 / 2 | `Radio_last_frame_us`、SBUS フラグのビット定義 |
| `wifi_telemetry.hpp` | 4 | `_pad` → `ftc` ＋ `failsafe` を新設。**パケット長 187 → 188、`NMPC_UDP_VER` 3 → 4** |
| `tools/nmpc_udp_client.py` | 6 | CSV 列に `ftc` / `failsafe`、`TEL_FMT` と `NMPC_UDP_VER` を更新 |

### 検証

- **ビルド成功。** `text 675776 / bss 352224`
- `static_assert`: `sizeof(NmpcTelPkt) == 188`、`offsetof(ftc)==14` / `offsetof(failsafe)==15` /
  `offsetof(cycles_since_reset)==16`、`NMPC_UDP_VER == 4`。**Python 側の `TEL_FMT` も 188 で一致**
- ⚠ **パケット長が変わったので、ファームとクライアントの `NMPC_UDP_VER` を必ず揃えること。**
  不一致のときクライアントは警告を出すが、**古いクライアントで新ファームのログを取ると列がずれる**
- η→box→duty の算術確認（`u_hover` 0.10–0.78 のすべてで `η=0 → duty = 0.00`）
- `Flight_mode` をチャンネル定数と比較している箇所が無いことを確認
  （**`Flight_mode == HOVERING` が 2 箇所残っていたのを `MODE_HOVERING` に修正**。
  `HOVERING` は radio.hpp の 8 なので、直さないと LED のホバリング表示が死んでいた）
- **実機での動作確認は未実施**

### 見つけて直したこと

1. **ビルドが壊れていた**（`REDCIRCLE` 未定義 4 箇所）
2. **ch5 が Flight_mode の選択にも使われていた** → 外した。放置すると FTC スイッチでモードが変わる
3. **`Flight_mode == HOVERING` が LED と `Merker_Hovering()` に残っていた** → `MODE_HOVERING` へ
4. **OverG のしきい値 800 m/s² が到達不能だった** → 150 m/s² へ

---

## 10. 確定したしきい値

| 項目 | 値 | 根拠 |
|---|---|---|
| 姿勢 | **60°** | スティック指令 ±54° に対し 6° のマージン |
| 角速度 | **300 deg/s** | 0831 の横転時は 500 deg/s 超 |
| 加速度 | **150 m/s²**（≈15 g） | 旧 800 は `LSM9DS1_16g`（最大 157 / 3 軸合成 272）で到達不能だった。誤検知で飛行中に止まる方が危険なので高めから |
| 受信途絶 | **200 ms** | SBUS 14 ms 周期の 14 フレーム分 |

**いずれも実測に基づいた値ではありません。** 係留ログで `acc_norm` と角速度の分布を見てから詰めてください。

---

## 11. 次にやること

1. **プロペラを外して机上確認**
   - スロットルを上げて 6 発が回る
   - ch5 ON → **FR だけ止まる**、他 5 発は回り続ける
   - LED が通常⇔全周青の交互になる
   - ch5 OFF → FR が復帰
   - テレメで `u_fr = 0.000`、`ftc = 1`、他の `u_*` が ON 前より上がっている
2. **クライアントを更新して起動**（`NMPC_UDP_VER` が 4 になっています）
3. **係留で `acc_norm` と角速度の分布を記録** → しきい値を確定
4. 健全飛行が安定してから、高度を取って FTC を試す
