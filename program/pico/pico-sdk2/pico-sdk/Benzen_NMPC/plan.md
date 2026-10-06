# 実機 Pico 2 W / Hexa-X 姿勢 NMPC 化 実装計画

> **2026-09-20 更新**: 本文は初回6発移植時の履歴であり、`N=20/NX=5` 等は古い。
> 現行の正は `nmpc_params.hpp` の **NX=8 / N=12 / NU=6 / tau_act=45 ms / DC0**。
> 状態は `[phi,theta,p,q,r,a_p,a_q,a_r]`、末尾3状態は最終PWM dutyから400 Hzで伝播する。
> 詳細と検証基準は `docs/20260920_nx8_n12_actuator_lag_implementation_plan.md` を参照。
> ホストモデル検証とクロスビルド後であり、Pico実測時間・通電・飛行は未検証。

対象: Raspberry Pi Pico 2 W で動作する Hexa-X ヘキサコプタのフライトコントローラ
目的: 姿勢制御を PID から NMPC へ置き換える(高度制御は PID/スティックのまま)
参照実装: `../../../../NMPC_simulation/nmpc_simulation.py`(Hexa-X 6発、実証済み)

---

## 0. 調査で確定した事実

- 既存 NMPC コア `nmpc.cpp` は `mpc::NU` / `mpc::NU_TOTAL` 経由で記述され、**ロジックは NU 非依存**。`4` のハードコードは無い。6発化の実体は2箇所のみ:
  - `nmpc_params.hpp` の `NU = 4 → 6`
  - `nmpc_model.cpp` の `model_f` / `model_B_const`(Quad固定式)を Hexa へ
- 型定義 `StateVec/InputVec/StateMat/InputMat`(NX×NU)は `nmpc_model.hpp`、
  `XStackVec/UStackVec/SuMat/HMat` は `nmpc.hpp` に既存。すべて `mpc::NU` 依存のため、
  **新規 `nmpc_types.hpp` は不要**(NU を変えれば全行列が自動 6発化される)。
- 既存 C++ NMPC は **絶対モード**(最適化変数 U は絶対 duty、box `[0,1]`、`U_ref` あり)。
  シミュの加算合成(PKG-C)は C++ には未実装。
  → Phase 3 はコア無改造で済む **絶対モード + `U_ref = ホバー baseline`** を採用
  (`U_ref` への引力で各モータが baseline 付近に収束、姿勢補正は差分的に作用。`B_c·1=0` より共通モードは baseline を維持)。
- 既存ミキサ `control.cpp` L900-905、出力は `set_duty_fr/fl/mr/ml/rr/rl`、
  各モータの baseline スラスト項は `T_ref * T_ref_coe`(`T_ref_coe = 0.6`)。
  → NMPC の `u_hover = T_ref * T_ref_coe`(既存の単位系をそのまま踏襲)。

シミュと実機 C++ のパラメータ乖離(転記が必要):

| | N | Q_p | Q_q | Q_r | R_u | terminal_scale |
|---|---|---|---|---|---|---|
| sim | 30 | 10 | 10 | 5 | 14 | 10 |
| C++ 旧 | 20 | 1 | 1 | 1 | 25 | 5 |
| 本実装 | 20 | 10 | 10 | 5 | 14 | 10 |

N は Pico のメモリ/演算量を優先し 20 を維持(Phase 0 で実測後に確定)。

---

## 1. Phase 1: モータ配置の逆算(最重要・完了)

実機ミキサ係数 `control.cpp` L900-905:

```
FR = T*coe + (-1.7655 P + 3.0562 Q - 1.7655 R)
MR = T*coe + (-3.5311 P + 0.0    Q + 1.7655 R)
RR = T*coe + (-1.7655 P - 3.0562 Q - 1.7655 R)
RL = T*coe + ( 1.7655 P - 3.0562 Q + 1.7655 R)
ML = T*coe + ( 3.5311 P + 0.0    Q - 1.7655 R)
FL = T*coe + ( 1.7655 P + 3.0562 Q + 1.7655 R)
```

シミュ `HexaModel`: 角度 `[90,-90,-30,150,30,-150]°`、s_yaw `[-1,+1,-1,+1,+1,-1]`(CW=-1, CCW=+1)。

### モータ対応表(逆算で確定)

| 実機 | 位置 | sim | sim (x, y) [×L] | sim s_yaw | 実機 R係数 | 実機 yaw 符号 s_real |
|---|---|---|---|---|---|---|
| FR | 前右 | M5 | (+0.866, +0.5) | +1 (CCW) | -1.7655 | **-1** |
| FL | 前左 | M3 | (+0.866, -0.5) | -1 (CW)  | +1.7655 | **+1** |
| MR | 右   | M1 | (0,      +1.0) | -1 (CW)  | +1.7655 | **+1** |
| ML | 左   | M2 | (0,      -1.0) | +1 (CCW) | -1.7655 | **-1** |
| RR | 後右 | M4 | (-0.866, +0.5) | +1 (CCW) | -1.7655 | **-1** |
| RL | 後左 | M6 | (-0.866, -0.5) | -1 (CW)  | +1.7655 | **+1** |

### 符号検証
- ロール `B_c[2,k] = -y_k T_max/Ix`、ピッチ `B_c[3,k] = +x_k T_max/Iy`:
  実機ミキサの P/Q 係数と **符号一致**(磁機の roll 比 MR:FR = |y|=1:0.5 = ミキサ 3.53:1.77 ≈ 2:1 とも整合)。
- ヨー: sim の s_yaw と実機 R 係数は **全モータ一貫して符号反転**。
  → 実機の測定 r(`Wr = -angular_rate_mdps[2]`)に合わせ、モデルのヨー列は
  `s_real = -s_sim`(= 実機 R 係数の符号)を採用する。

### 実装(本計画)
- `model_B_const()` を **実機モータ順序 `[FR, FL, MR, ML, RR, RL]`** で構築。
  これにより NMPC 出力 `U_opt(0..5)` をそのまま `FR_duty..RL_duty` に割り当てられる。
- 列の (x, y, s_real):

| idx | motor | x [×L] | y [×L] | s_real |
|---|---|---|---|---|
| 0 | FR | +0.866 | +0.5 | -1 |
| 1 | FL | +0.866 | -0.5 | +1 |
| 2 | MR | 0.0    | +1.0 | +1 |
| 3 | ML | 0.0    | -1.0 | -1 |
| 4 | RR | -0.866 | +0.5 | -1 |
| 5 | RL | -0.866 | -0.5 | +1 |

- 初期化時 assert: `B_c · 1 = 0`(全6発均等で姿勢トルク0)、`rank(B_c[2:5,:]) = 3`。
- **ヨー符号は実機通電テスト(Phase 4)で最終確認**(机上では一貫性のみ確認済)。

---

## 2. Phase 0: メモリ/演算量見積もり

- N=20 で `NU_TOTAL = 120`:
  - `Su` 100×120 = 48KB, `M(H)` 120×120 = 57.6KB, LLT `L` ≈ 57.6KB
  - `NmpcContext` ≈ 200KB(+ `nmpc.cpp` の `static SuMat Su_w` 48KB)
  - RP2350 SRAM 520KB に乗る見込み。`.map` で実測する
- RAM 逼迫時は `N=20→15`(`NU_TOTAL=90`, H≈32KB)
- 演算量: Cholesky フル再構築 O(120³) は `reset_period=100` で 100周期に1回。
  通常周期は SR2 差分 O(120²)。実時間は `time_us_32()` で Phase 4 実測(目標 < 10ms)

---

## 3. Phase 2: NMPC コア 6発拡張

- `nmpc_params.hpp`: `NU=6`、cost を sim 値へ(Q_p=Q_q=10, Q_r=5, R_u=14, terminal_scale=10)
- `nmpc_model.cpp`:
  - `model_f`: `Bu = B_c·u` 方式(sim `HexaModel.f` と同形)で6入力化。
    空力ダンピング `Cp/Cq/Cr`・`smooth_abs` は現状維持(sim と一致)
  - `model_B_const`: Phase 1 の実機順序・s_real で6発化
  - `model_jacobians`: A_c は NU 非依存で流用、`model_B_const` 呼び出しのみ
- `nmpc.cpp` / `nmpc.hpp`: 無改造(`mpc::NU` 経由で自動6発化)
- 初期化時サニティ(`nmpc_model_sanity()`): `B_c·1≈0`, rank=3

---

## 4. Phase 3: control.cpp 統合(`#ifdef USE_NMPC`)

`USE_NMPC` 定義で PID 版 / NMPC 版を切替(問題時に即戻せる)。

- グローバル `nmpc::NmpcContext`(static, BSS)を `control_init()` で `nmpc_init()`
- `angle_control()`(core1, 100Hz):
  - `kalman_filter()` → `Phi/Theta/Psi`
  - `x0 = [Phi-Phi_bias, Theta-Theta_bias, Wp-Pbias, Wq-Qbias, Wr-Rbias]`
  - `x_ref = [Phi_ref, Theta_ref, 0, 0, Psi_ref]`(既存スティック式を流用、Psi_ref はヨー rate 目標)
  - `u_hover = clip(T_ref * T_ref_coe, 0, 0.95)`、`U_ref = tile(u_hover)`
  - `nmpc_step()` → `U_opt(0..5)` を グローバル `FR_duty..RL_duty` に書込
  - 非武装(`T_ref/BATT < Flight_duty`)時は `nmpc_reset_request()` + バイアス保持
- `rate_control()`(core0, 400Hz):
  - NMPC 版では PID/ミキサを `#ifndef` で無効化
  - グローバル `FR_duty..RL_duty`(100Hz で更新)を clamp + safety(Disable/OverG) して
    `set_duty_*` で 400Hz ZOH 出力(既存セーフティを維持)
- フェイルセーフ: NMPC が NaN/Inf → motor_stop / 前回値保持。OverG/Arm/LockMode は維持

---

## 5. Phase 4–6: 実機検証(Cursor はログ/パラメータ支援)

- Phase 4: プロペラ無し→傾きで正しいモータ反応(符号検証)、iter<10ms 実測、拘束1軸追従、係留ホバー
- Phase 5: 自由飛行(健全機)、スティック追従、外乱復元、sim S1 比較
- Phase 6(将来): FDI + `set_eta` 相当で B列更新、1発故障維持、sim S3 比較

---

## 6. 未確定事項(Phase 0/1/4 で確定)

1. ヨー軸の符号(実機 r と モデル B_c ヨー列の一致)
2. T_max 実機値 / `T_ref_coe=0.6` の物理意味(ホバー duty 実測から逆算)
3. N 最終値(20 か 15、メモリ実測次第)
4. モータ出力レート(現状 400Hz ZOH + 100Hz NMPC)
5. コスト重みの実機再調整(sim N=30 系からの転記初期値)
