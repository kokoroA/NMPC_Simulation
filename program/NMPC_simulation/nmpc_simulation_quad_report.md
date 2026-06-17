# nmpc_simulation_quad.py 詳細解説レポート

**対象ファイル**: `nmpc_simulation_quad.py`（2327 行）  
**更新日**: 2026-05-27  
**目的**: Quad-X 配置クアッドコプタの耐故障姿勢＋高度制御を NMPC (ADMM + SR2) で検証する Python シミュレータ  
**参照実装**: `sample/nmpc.cpp`, `nmpc_model.cpp`, `nmpc_params.hpp`（Pico 2 移植前オフライン検証用）  
**関連**: ヘキサ版は `nmpc_simulation.py`（NU=6、Hexa-X 配置、同一 PKG 構成）。ヘキサ解説は `nmpc_simulation_report.md`。

---

## 1. 概要

4 モータ・5 状態（φ, θ, p, q, r）の NMPC を凝縮形で解き、プラントを **1 kHz** で積分しながら NMPC は **100 Hz**（ZOH）で更新する **マルチレート** 構成。設計思想・PKG 構成は `nmpc_simulation.py` と同一で、機体配置と入力次元のみクアッド向けに差し替えている。

NMPC コア（`QuadModel`, `AdmmNmpcQuad`）は理想化モデルのまま据え置き、**プラント・観測・実行ループ** に実機相当の非理想性を載せることで、実機適用性を評価できる。

### 1.1 ヘキサ版との主な差分

| 項目 | ヘキサ (`nmpc_simulation.py`) | クアッド (`nmpc_simulation_quad.py`) |
|------|------------------------------|-------------------------------------|
| モータ数 `NU` | 6 | **4** |
| 配置 | Hexa-X | **Quad-X**（ArduPilot/PX4 標準） |
| 冗長性（null space） | 3 次元 | **1 次元**（平均モードのみ） |
| 凝縮 H 行列 | 120×120 | **80×80** |
| `u_hover_nom` | ≈ 0.120 | **≈ 0.180** |
| `pwm_ascent` | 0.145 | **0.215**（4 機・PKG-E マージン込み） |
| `R_u` | 14.0 | **21.0**（14×6/4、モータ当たり制御権限の補正） |
| 完全故障 (η=0) | 残存 5 機でホバー可能 | **幾何的にホバー+零モーメント同時達成不可** |
| 故障シナリオ | 完全・部分故障を幅広く | **部分故障中心**（S5 のみ完全故障デモ） |

### 1.2 統合機能一覧（PKG-A〜E）

| パッケージ | 内容 |
|-----------|------|
| **PKG-A** | Bumpless Transfer（ascent→control）：`w_ref` ランプ、`Kd` フェードイン、`z_ref` ランプ |
| **PKG-B** | 高度 PID 再チューニング（Kp=2.0, Kd=1.0）で切替直後の bang-bang 抑制 |
| **PKG-C** | 加算合成：`u_cmd = u_baseline + Δu`（NMPC は差分のみ最適化） |
| **PKG-D** | 故障時 Bumpless：`u_baseline` ramp、PID・`z_ref` 再起動、`i_z` 減衰 |
| **PKG-E1** | モータ一次遅れ（τ=50 ms）+ アクチュエータ遅延（2 ms） |
| **PKG-E2** | プラント側物理パラメータ誤差（NMPC モデルとの不一致） |
| **PKG-E3** | 空力推力モデル（プロペラ前進比効果） |
| **PKG-E4** | センサ遅延（IMU 5 ms / 高度 20 ms） |
| **PKG-E5** | 水平移動（x, y, u_v, v_v）— プラント 11 次元 |
| **PKG-E6** | プロペラジャイロ（簡易版） |
| **PKG-E7** | 地面効果（低空 +15% 推力） |
| **PKG-E8** | ESC PWM 12-bit 量子化 |
| **ノイズ** | 真値 `x_true` / 観測 `x_meas` 分離（プロセス・観測ノイズ） |
| **外乱** | world 力・body トルクのスケジュール注入 |
| **Λ スケーリング** | `B_eff = B_nom · diag(η)`（FDI 既知時） |

---

## 2. アーキテクチャ

```mermaid
flowchart TB
    subgraph ctrl["制御層 (100 Hz, ZOH)"]
        PID["AltitudePID\nPKG-B"]
        NMPC["AdmmNmpcQuad\n理想モデル (PKG-C 差分)"]
    end

    subgraph sense["観測層 (1 kHz)"]
        Noise["NoiseModel\nPKG-E4 遅延 + ノイズ"]
    end

    subgraph act["アクチュエータ (1 kHz)"]
        ESC["PKG-E8 量子化"]
        Motor["PKG-E1 遅延 + 一次遅れ"]
    end

    subgraph plant_block["PlantModel (1 kHz, RK4, PKG-E)"]
        Plant["x11 = φ,θ,p,q,r,z,w,x,y,u_v,v_v\np_plant ≠ p_nmpc"]
    end

    z_ref["z_ref ランプ\nPKG-A/D"] --> PID
    phi_ref["φ_ref"] --> NMPC
    Plant -->|x_meas[:5]| NMPC
    Plant -->|z,w 遅延観測| PID
    PID -->|T_cmd → u_baseline| Sum["PKG-C\nu_cmd = u_base + Δu"]
    NMPC -->|Δu| Sum
    Sum --> ESC --> Motor
    Motor -->|u_actual| Fault["η ⊙ u_actual"]
    Fault --> Plant
    FM["FaultManager"] -->|η, FDI| NMPC
    FM --> Fault
    DM["DisturbanceManager"] --> Plant
```

### 2.1 フェーズ遷移（`run_scenario` 内）

```
t = 0        地上 z=0、phase=ascent（全機 PWM=pwm_ascent 固定）
             NMPC は動作するが出力は ascent 中は無視
             plant.reset_motors(pwm_ascent)  ← PKG-E1 初期化

t ≈ 3 s      z ≤ z_switch = z_target + z_switch_offset (= -1.0 m, 高度 1 m)
             → phase=control、alt_pid.reset()
             → PKG-A: w_at_switch 保存、kd_fadein / w_ref_ramp 開始
             → PKG-C: nmpc.U=0（Δu warm-start）、加算合成 ON

t = t_switch + 2.0 s
             → plan_faults 適用（S1: なし、S2a: M3 η=0.7、…）
             → FDI 既知: B_eff 更新、box 再設定、SR2 リセット
             → PKG-D: u_baseline ramp、bumpless restart、i_z×0.5

t = t_fault + 5.0 s
             → phi_ref = 0.2 rad ステップ（roll 応答観測）

t = 15 s     シミュレーション終了
```

---

## 3. 座標系と記号

### 3.1 NED（North-East-Down）

| 変数 | 符号 | 意味 |
|------|------|------|
| z | 下向き正 [m] | 地上 z=0、高度 2 m → **z = -2.0** |
| w | 下向き正 [m/s] | 落下で w > 0 |
| T_cmd | [N] | 機体推力総和コマンド（上向き） |

プロットの「高度」は **-z**（上向き正）で表示。水平位置は **x_pos, y_pos**（NED、idx 7, 8）。

### 3.2 Quad-X モータ配置

| モータ | 名称 | 方位 [°] | 回転 | x_motor [m] | y_motor [m] | s_yaw |
|--------|------|----------|------|-------------|-------------|-------|
| M1 (idx 0) | FR 前右 | +45 | CCW (+1) | +L/√2 | +L/√2 | +1 |
| M2 (idx 1) | RL 後左 | −135 | CCW (+1) | −L/√2 | −L/√2 | +1 |
| M3 (idx 2) | FL 前左 | −45 | CW (−1) | +L/√2 | −L/√2 | −1 |
| M4 (idx 3) | RR 後右 | +135 | CW (−1) | −L/√2 | +L/√2 | −1 |

`L = arm_length = 0.050 m`。座標は機体重心を原点とした body 座標。

**PKG-C の成立条件**: `B_c · 1` が 3 軸とも 0 → 平均モード（高度）と差分モード（姿勢）が分離。null space は **1 次元**（±[0.5, 0.5, 0.5, 0.5] 近傍 = 全機均等 duty）。

**冗長性の限界**: `rank(B_c[2:5,:]) = 3` だが `NU = 4` のため、1 機完全停止（η=0）すると「ホバー推力配分 + 零モーメント」を同時に満たす非自明解が存在しない（`sanity_checks` 項目 7b で検証）。

---

## 4. モジュール構成一覧

| # | クラス / 関数 | 行付近 | 役割 |
|---|---------------|--------|------|
| — | `make_run_img_dir` | 51 | `img/<timestamp>/` 作成 |
| — | `_cholesky_lower` | 63 | Cholesky ラッパ |
| 1 | `Params` | 72 | 全パラメータ（PKG-A〜E 含む） |
| 2 | `QuadModel` | 215 | 姿勢 CT モデル・Jacobian・Λ スケーリング |
| 2.5 | `PlantModel` | 362 | 11 状態プラント RK4 + PKG-E1/E2/E3/E5/E6/E7 |
| 2.6 | `AltitudePID` | 521 | PID + 重力補償 + PKG-A bumpless |
| — | `t_cmd_to_u_hover` | 586 | T_cmd → per-motor duty（生存機均等） |
| 2.7 | `DisturbanceManager` | 606 | 力/トルク外乱 |
| 2.8 | `NoiseModel` | 642 | ノイズ + PKG-E4 センサ遅延 |
| 3 | `AdmmNmpcQuad` | 751 | 凝縮 NMPC（PKG-C 加算合成対応） |
| 4 | `FaultManager` | 1038 | η(t) + `apply_faults` |
| 5 | `Logger` | 1088 | 1 kHz ログ + 5×4 プロット（4 モータ表示） |
| 6 | `sanity_checks` | 1324 | 起動時 24 項目（Quad 固有検証含む） |
| 7 | `run_scenario` | 1764 | マルチレート + 全 PKG 統合ループ |
| 8 | `main` | 2153 | S1〜S8 一括実行 |

---

## 5. Params（パラメータ）

### 5.1 物理パラメータ（NMPC が信じる公称値）

| 記号 | 変数 | 値 | 意味 |
|------|------|----|------|
| m | `mass` | 0.200 kg | 機体質量 |
| g | `gravity` | 9.81 m/s² | 重力 |
| Ix/Iy/Iz | 各慣性 | 6.10e-3 / 6.53e-3 / 1.16e-2 kg·m² | 主慣性モーメント |
| L | `arm_length` | 0.050 m | モーター–重心距離 |
| T_max | `T_max` | 2.725 N | 1 モータ最大推力 |
| C_QT | `C_QT` | 0.03614 m | 反トルク/推力比 |

**プロパティ**: `u_hover_nom = m·g / (4·T_max) ≈ 0.180`（ヘキサの 0.120 より高い：モータ当たり推力負担が大きい）

### 5.2 NMPC・マルチレート

| 変数 | 値 | 説明 |
|------|----|------|
| `N` | 20 | 予測ホライズン |
| `dt` | 0.01 s | NMPC 離散化周期（Sx/Su/H） |
| `dt_sim` | 0.001 s | プラント RK4 周期（1 kHz） |
| `nmpc_decimation` | 10 | NMPC 実効 100 Hz |
| `NX`, `NU` | 5, **4** | NMPC 状態・入力次元 |
| `NX_plant` | **11** | プラント状態次元（PKG-E5） |
| `NUT` | **80** | 凝縮 H 行列サイズ（NU·N = 4·20） |

### 5.3 高度 PID（PKG-B）

| 変数 | 値 | 説明 |
|------|----|------|
| `z_ref_default` | -2.0 m | 2 m ホバー目標（NED） |
| `Kp_z` | **2.0** | P ゲイン |
| `Ki_z` | 0.3 | 積分 |
| `Kd_z` | **1.0** | D ゲイン（切替キック抑制） |
| `i_z_limit` | 0.2 N | 積分クランプ |
| `T_cmd_min_ratio` | 0.3 | 推力下限 = 0.3·mg |
| `trim_clip_cos` | 0.5 | cos(φ)cos(θ) 下限 |

### 5.4 Open-loop 離陸・Bumpless（PKG-A）

| 変数 | 値 | 説明 |
|------|----|------|
| `pwm_ascent` | **0.215** | 上昇 per-motor duty（4 機・PKG-E2/E3 マージン、ヘキサ 0.145 より大） |
| `z_switch_offset` | **1.0 m** | 目標より 1 m 手前で PID 切替 |
| `z_ramp_duration` | **3.0 s** | z_ref ランプ時間 |
| `u_ref_lpf_tau` | **0.0** | PKG-C 移行で LPF 無効 |
| `kd_fadein_duration` | 0.5 s | D 項フェードイン |
| `w_ref_ramp_duration` | 1.0 s | w_ref ランプ |

### 5.5 故障時 Bumpless（PKG-D）

| 変数 | 値 | 説明 |
|------|----|------|
| `fault_baseline_ramp_duration` | 0.3 s | u_baseline の故障時線形 ramp |
| `fault_iz_attenuation` | 0.5 | 故障時 PID 積分項減衰 |

### 5.6 コスト・RTI

| 変数 | 値 | 説明 |
|------|----|------|
| `Q_phi/Q_theta` | 100 | 姿勢角 |
| `Q_p/Q_q/Q_r` | 1 | 角速度 |
| `R_u` | **21.0** | 入力ペナルティ（14×6/4） |
| `terminal_scale` | 5 | 終端 Q 倍率 |
| `admm_max_iter` | 1 | ADMM 1 反復 |
| `reset_period` | 100 | フル再構築間隔 [NMPC 周期] |

### 5.7 ノイズ・PKG-E

ヘキサ版と同一デフォルト（`noise_enable=True`, PKG-E1〜E8 の各フラグ・比率も同値）。詳細は `nmpc_simulation_report.md` §5.7〜5.8 を参照。

---

## 6. QuadModel（姿勢ダイナミクス）— NMPC 内部モデル

FDI 既知時のみ `set_eta(η)` で `B_eff = B_nom · diag(η)` を更新。アルゴリズムは `HexaModel` と同一。

### 6.1 連続時間モデル

状態 **x = [φ, θ, p, q, r]**。`|v|ε = √(v² + ε²)` で C² 化（`damping_eps = 1e-3`）。

### 6.2 B_c 行列（5×4）

```
B_c[2,k] = −y_k · T_max / Ix     (ロール)
B_c[3,k] = +x_k · T_max / Iy     (ピッチ)
B_c[4,k] = s_k · C_QT · T_max / Iz  (ヨー)
```

### 6.3 クアッド固有：完全故障の幾何学

部分故障（η > 0）では `rank(B_eff) = 3` を維持し FTC 可能。完全故障（η = 0）では：

- 残存 3 機の `B_eff[2:5,:]` のランクは依然 3（姿勢方向は張れる）
- しかし拡張行列 `[B_alive; 1^T]` のランクが 3 のため、「零モーメント + ホバー推力」の同時達成は **u = 0 のみ** → 実質飛行不能

これが S5（M3 η=0 デモ）の理論的根拠である。

---

## 7. PlantModel（11 状態プラント）— PKG-E

### 7.1 状態ベクトル（PKG-E5）

**x_plant = [φ, θ, p, q, r, z, w, x_pos, y_pos, u_vel, v_vel]**（11 次元）

姿勢部分は `QuadModel(p_plant)` を流用。`p_plant` は PKG-E2 で公称値からずらした「真」物理パラメータ。

### 7.2 推力・並進・PKG-E6〜E8

ヘキサ版と同式。違いは `T_total_raw = T_max_plant · Σ u` の和が **4 項**である点と、プロペラジャイロの `Σ s_i·u_i` が Quad-X の CCW/CW 配置でホバー時に相殺されること（1 機停止で非ゼロになりうる）。

---

## 8. PKG-C: 加算合成制御

### 8.1 制御構造

```
u_baseline = t_cmd_to_u_hover(T_cmd, η)   # 高度 PID → 生存機均等配分
Δu         = nmpc.step(x_meas[:5], ..., u_baseline=u_baseline)
u_cmd      = clip(u_baseline + Δu, 0, 1)
```

NMPC の box 制約（動的更新）:

```
Δu_min[k] = U_min − u_baseline[k]
Δu_max[k] = U_max − u_baseline[k]   （故障機は 0 固定）
```

内部ロールアウト・Jacobian は `u_actual = Δu + u_baseline` で評価。`U_ref` はゼロベクトル（差分の基準は 0）。

### 8.2 ヘキサとの違い

加算合成の数学的構造は同一。クアッドでは `t_cmd_to_u_hover` が 3 機生存時に `mg/(3·T_max)` へ再配分するため、1 機故障後の baseline がより大きくなり、残存機のサチュレーションリスクが高まる。

---

## 9. AdmmNmpcQuad（凝縮 NMPC + ADMM + SR2）

`sample/nmpc.cpp` の Python 鏡像。クラス名のみ `AdmmNmpcHex` → `AdmmNmpcQuad`、内部モデルが `QuadModel`。

### 9.1 1 周期の処理フロー

1. warm-start シフト（`U`, `z`, `λ` を 1 ステップ前へ）
2. `reset_period` ごと、または SR2 失敗時に `_full_rebuild_sx_su` + `_full_rebuild_chol`
3. それ以外は TR1（Broyden rank-1）で `Sx`, `Su` 更新 → SR2 で `H` 更新
4. 勾配 `g = Su^T Q_bar (Sx·x0 − X_ref) − R_bar·U_ref`
5. ADMM 1 反復（box 射影）
6. 先頭 `NU` 成分を `Δu` として返却

### 9.2 計算規模

| 行列 | サイズ（クアッド） | サイズ（ヘキサ） |
|------|-------------------|-----------------|
| Sx | 100×5 | 100×5 |
| Su | 100×**80** | 100×120 |
| H | **80×80** | 120×120 |

クアッド版は H の Cholesky が約 2.25 倍軽く、実時間性のマージンが大きい。

---

## 10. 観測・外乱・故障

### 10.1 NoiseModel

- プロセスノイズ: 各 RK4 ステップで `σ · √(dt_sim)` を状態に加算
- 観測ノイズ: IMU（φ, θ, p, q, r）と高度（z, w）に独立ガウス
- PKG-E4: `deque` による IMU / 高度の遅延バッファ

### 10.2 FaultManager

- `schedule`: `(t_fail, motor_idx, eta_target)` のリスト
- `update(t, dt)`: η の時間発展 + 発火イベント返却
- `apply_faults`: プラン駆動の即時故障注入（`run_scenario` の `plan_faults`）

### 10.3 FDI 既知 vs 未知

| モード | NMPC 側の扱い |
|--------|--------------|
| `fdi_known=True` | `QuadModel.set_eta(η)` で `B_eff` 更新、η≈0 で `u_max[k]=0`、SR2 リセット |
| `fdi_known=False` | B_eff は公称のまま（モデル不一致が残る） |

**注意**: `u_max` の η 比例縮減は廃止（B_eff に反映済みの二重スケーリング防止）。

---

## 11. Logger とメトリクス

### 11.1 ログ内容

1 kHz で `t, x_true(7+), x_meas, u_cmd(4), u_applied, η(4), iter_us, was_reset, x_ref, z_ref, T_cmd/T_trim/T_fb, F_d, tau_d` を記録。

### 11.2 プロット（5×4）

- 行 0: φ, θ, 姿勢誤差, NMPC 反復時間 [μs]
- 行 1: p, q, r, SR2 reset フラグ
- 行 2: u_cmd（M1,M2 / M3,M4 / 全 4 / Σu_app）
- 行 3: u_app、η、累積サチュ率
- 行 4: 高度（-z）、w、PID 推力内訳、外乱

11 次元プラント使用時はタイトルに水平 drift（最終位置・最大距離）を注釈。

### 11.3 compute_metrics

| メトリクス | 意味 |
|-----------|------|
| `rms_phi`, `rms_theta` | 姿勢追従 RMS [rad] |
| `rms_z_err` | 高度誤差 RMS [m] |
| `peak_p/q/r`, `peak_w` | 角速度・鉛直速度ピーク |
| `sat_rate` | いずれかモータが 0/1 付近の割合 |
| `mean/p50/p99_iter_us` | NMPC 反復時間 [μs]（iter_us>0 のみ） |

---

## 12. sanity_checks（起動時 24 項目）

`main()` 冒頭で実行。Quad 固有の検証を含む。

| # | 内容 |
|---|------|
| 1–5 | B_c 数値、M1/M3 単独 duty のトルク符号、ホバートリム、総推力 = mg |
| 6 | `rank(B_c[2:5,:])=3`、null space ≈ ±[0.5,…,0.5] |
| 7 | `set_eta` による B_eff スケーリング |
| 7b | **η=0 完全故障でホバー+零モーメント不可** |
| 8–10 | AltitudePID ホバー、重力補償、アンチワインドアップ |
| 11 | PlantModel 姿勢ホバー（p_plant の duty 使用） |
| 12–13 | DisturbanceManager、`t_cmd_to_u_hover`（3 機故障時） |
| 14–16 | PKG-C 加算合成、box、PKG-D ramp |
| 17–24 | PKG-E1〜E8 |

---

## 13. run_scenario（メインシミュレーションループ）

### 13.1 マルチレート構造

```python
for n in range(n_sim_steps):          # dt_sim = 1 ms
    if n % dec == 0:                  # 100 Hz
        # 高度 PID + NMPC（control フェーズのみ）
    # PKG-E8 量子化 → PKG-E1 → η⊙u → Plant RK4 → ノイズ
    logger.log(...)
```

### 13.2 主要な分岐

| 条件 | 動作 |
|------|------|
| `phase == "ascent"` | `u_cmd = pwm_ascent`（NMPC 出力無視） |
| `phase == "control"` | PID + 加算合成 NMPC |
| `z ≤ z_switch` | ascent → control 遷移、PKG-A/C 初期化 |
| `t ≥ t_switch + fault_delay` | `plan_faults` 適用 |
| `events` + `fdi_known` | PKG-D bumpless + B_eff 更新 |

---

## 14. シナリオ一覧（main: S1〜S8）

共通プラン（`run_ftc_plan`）:

- 地上 z=0 から `pwm_ascent=0.215` で open-loop 上昇
- 高度 **1 m**（`z_switch = -1.0`）で PID 切替 → `z_ref` を 3 s かけて 2 m へ
- 切替 **+2 s** で故障注入
- 故障 **+5 s** で `phi_ref = 0.2 rad` ステップ
- シミュレーション時間 **15 s**

| ID | 内容 | FDI | 故障 |
|----|------|-----|------|
| **S1** | 健全 | known | なし |
| **S2a** | M3 部分故障 η=0.7 | known | M3 |
| **S2b** | M3 部分故障 η=0.7 | **unknown** | M3 |
| **S3a** | M3 部分故障 η=0.5 | known | M3 |
| **S3b** | M3 部分故障 η=0.5 | unknown | M3 |
| **S4** | M3 厳しい部分 η=0.3 | known | M3 |
| **S5** | M3 **完全故障** η=0（制御不能デモ） | known | M3 |
| **S6** | 鉛直風突風 F_z=+0.5 N | known | なし |
| **S7** | ロールトルク突風 τ_x=+0.02 Nm | known | なし |
| **S8** | M3 η=0.5 + τ_x 突風 | known | M3 |

**シナリオ設計方針**: クアッドは冗長性が 1 次元のため、実用的 FTC 評価は **M3（前左, CW）の部分故障** を中心に構成。S5 のみ敢えて完全故障を走らせ、幾何的限界を可視化する。

---

## 15. 実行方法

```bash
cd /path/to/NMPC_simulation
python nmpc_simulation_quad.py
```

### 15.1 出力

| 出力 | パス |
|------|------|
| 各シナリオ PNG | `img/<YYYYMMDD_HHMMSS>/sim_s*.png` |
| メトリクス CSV | `img/<timestamp>/scenario_metrics.csv` |
| 比較棒グラフ | `img/<timestamp>/scenario_comparison.png` |
| コンソール | sanity_checks 結果 + S1〜S8 サマリ表 |

### 15.2 環境

- Python 3 + NumPy + Matplotlib
- `MPLBACKEND` 未設定時は Qt5Agg → TkAgg → MacOSX → Agg の順で自動選択
- 終了時 `plt.show()`（Agg では no-op）

---

## 16. ヘキサ版レポートとの対応

| 本レポート節 | `nmpc_simulation_report.md` |
|-------------|----------------------------|
| §6 QuadModel | §6 HexaModel |
| §9 AdmmNmpcQuad | §9 AdmmNmpcHex |
| §14 シナリオ | §14 シナリオ（故障パターンが Quad 向けに再編） |
| PKG-E 詳細 | §7, §5.8（同一実装） |

アルゴリズム（ADMM、SR2、TR1、PKG-A〜E）の詳細式・コード断片はヘキサ版レポートを併読するとよい。本ファイルは **クアッド固有の幾何・パラメータ・シナリオ** に焦点を当てている。

---

## 17. ファイル先頭 docstring（要約）

```1:18:nmpc_simulation_quad.py
"""
nmpc_simulation_quad.py
=======================
Quad-X 配置 クアッドコプタ向け 耐故障 NMPC シミュレータ。

設計は nmpc_simulation.py (Hexa-X 6 発版) と同一。差分は:
    - モータ数 NU=6 → NU=4
    - 機体配置 Hexa-X → Quad-X (45°/-135°/-45°/135°、CCW-CCW-CW-CW)
    - 冗長性: null space 3 次元 → 1 次元(完全故障は幾何的に対処不可)
    - 凝縮 H 行列サイズ 120×120 → 80×80
    - シナリオ S2/S3 は部分故障のみで構成(完全故障は飛行不能)

機構 (PKG-A 〜 PKG-D) はそのまま継承:
    PKG-A: Bumpless Transfer (ascent → control)
    PKG-B: PID gain (Kp_z=2.0, Kd_z=1.0)
    PKG-C: 加算合成 (u_cmd = u_baseline + Δu)
    PKG-D: 故障時 Bumpless Handling (u_baseline ramp + PID restart)
"""
```
