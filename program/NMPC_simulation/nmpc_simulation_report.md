# nmpc_simulation.py 詳細解説レポート（最新版）

**対象ファイル**: `nmpc_simulation.py`（2322 行）  
**更新日**: 2026-06-02（N=30、PKG-F、案B R スケーリング、S4 変更を反映）  
**目的**: Hexa-X 配置ヘキサコプタの耐故障姿勢＋高度制御を NMPC (ADMM + SR2) で検証する Python シミュレータ  
**参照実装**: `sample/nmpc.cpp`, `nmpc_model.cpp`, `nmpc_params.hpp`（Pico 2 移植前オフライン検証用）  
**関連**: クアッド版は `nmpc_simulation_quad.py`（NU=4、Quad-X 配置、同一 PKG 構成）

---

## 1. 概要

6 モータ・5 状態（φ, θ, p, q, r）の NMPC を凝縮形で解き、プラントを **1 kHz** で積分しながら NMPC は **100 Hz**（ZOH）で更新する **マルチレート** 構成。

NMPC コア（`HexaModel`, `AdmmNmpcHex`）は理想化モデルのまま据え置き、**プラント・観測・実行ループ** に実機相当の非理想性を載せることで、実機適用性を評価できる。

### 1.1 統合機能一覧

| パッケージ | 内容 |
|-----------|------|
| **PKG-A** | Bumpless Transfer（ascent→control）：`w_ref` ランプ、`Kd` フェードイン、`z_ref` ランプ |
| **PKG-B** | 高度 PID 再チューニング（Kp=2.0, Kd=1.0）で切替直後の bang-bang 抑制 |
| **PKG-C** | 加算合成：`u_cmd = u_baseline + Δu`（NMPC は差分のみ最適化） |
| **PKG-D** | 故障時 Bumpless：`u_baseline` ramp、PID・`z_ref` 再起動、`i_z` 減衰 |
| **PKG-E1** | モータ一次遅れ（τ=50 ms）+ アクチュエータ遅延（2 ms） |
| **PKG-E2** | プラント側物理パラメータ誤差（NMPC モデルとの不一致） |
| **PKG-E3** | 空力推力モデル（プロペラ前進比効果、`aero_factor_min/max` でクリップ） |
| **PKG-E4** | センサ遅延（IMU 5 ms / 高度 20 ms） |
| **PKG-E5** | 水平移動（x, y, u_v, v_v）— プラント 11 次元 |
| **PKG-E6** | プロペラジャイロ（簡易版） |
| **PKG-E7** | 地面効果（低空 +`ge_max_boost` 推力、`ge_height` で減衰） |
| **PKG-E8** | ESC PWM 12-bit 量子化 |
| **PKG-F** | FDI 未知時：制御層は `η_control=1`、box 全開放；プラントは真 `η` |
| **案B** | `set_rbar_from_eta`：故障機ほど `R_u/η` を大きくし Δu を抑制 |
| **ノイズ** | 真値 `x_true` / 観測 `x_meas` 分離（プロセス・観測ノイズ、オプションジャイロバイアス RW） |
| **外乱** | world 力・body トルクのスケジュール注入（水平力は PKG-E5 でプラントに反映） |
| **Λ スケーリング** | `B_eff = B_nom · diag(η)`（FDI 既知時） |

---

## 2. アーキテクチャ

```mermaid
flowchart TB
    subgraph ctrl["制御層 (100 Hz, ZOH)"]
        PID["AltitudePID\nPKG-B"]
        NMPC["AdmmNmpcHex\n理想モデル (PKG-C 差分)"]
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
    PID -->|T_cmd → u_baseline\nPKG-F: η_control| Sum["PKG-C\nu_cmd = u_base + Δu"]
    NMPC -->|Δu| Sum
    Sum --> ESC --> Motor
    Motor -->|u_actual| Fault["η ⊙ u_actual"]
    Fault --> Plant
    FM["FaultManager"] -->|η, FDI| NMPC
    FM --> Fault
    DM["DisturbanceManager"] --> Plant
```

### 2.1 フェーズ遷移（run_scenario 内）

```
t = 0        地上 z=0、phase=ascent（全機 PWM=pwm_ascent 固定）
             NMPC は動作するが出力は ascent 中は無視
             plant.reset_motors(pwm_ascent)  ← PKG-E1 初期化

t ≈ 1.2 s    z ≤ z_switch = z_target + z_switch_offset (= -1.0 m, 高度 1 m)
             → phase=control、alt_pid.reset()
             → PKG-A: w_at_switch 保存、kd_fadein / w_ref_ramp 開始
             → PKG-C: nmpc.U=0（Δu warm-start）、加算合成 ON

t = t_switch + 2.0 s
             → plan_faults 適用（S1: なし、S2a: M5 η=0.5、…）
             → FDI 既知: B_eff 更新、box 再設定、SR2 リセット、案B rbar 更新
             → PKG-D: u_baseline ramp、bumpless restart、i_z×0.5

t = t_fault + 5.0 s
             → phi_ref = 0.0 rad（PLAN_PHI_STEP、姿勢参照の同期イベント）

t = 15 s     シミュレーション終了
```

> 切替時刻は上昇 PWM・プラント非理想（E2/E3）に依存し、おおよそ **1.2〜1.3 s**（高度 1 m 到達）となる。

---

## 3. 座標系と記号

### 3.1 NED（North-East-Down）

| 変数 | 符号 | 意味 |
|------|------|------|
| z | 下向き正 [m] | 地上 z=0、高度 2 m → **z = -2.0** |
| w | 下向き正 [m/s] | 落下で w > 0 |
| T_cmd | [N] | 機体推力総和コマンド（上向き） |

プロットの「高度」は **-z**（上向き正）で表示。水平位置は **x_pos, y_pos**（NED、idx 7, 8）。

### 3.2 Hexa-X モータ配置

| モータ | 方位 | 回転 | x_motor [m] | y_motor [m] | s_yaw |
|--------|------|------|-------------|-------------|-------|
| M1 | +90° (右横) | CW (−1) | 0 | +0.05 | −1 |
| M2 | −90° (左横) | CCW (+1) | 0 | −0.05 | +1 |
| M3 | −30° (前左) | CW (−1) | +0.043 | −0.025 | −1 |
| M4 | +150° (後右) | CCW (+1) | −0.043 | +0.025 | +1 |
| M5 | +30° (前右) | CCW (+1) | +0.043 | +0.025 | +1 |
| M6 | −150° (後左) | CW (−1) | −0.043 | −0.025 | −1 |

**PKG-C の成立条件**: `B_c · 1` が 3 軸とも 0 → 平均モード（高度）と差分モード（姿勢）が分離。null space は 3 次元（ヘキサは 6 入力・ランク 3）。

---

## 4. モジュール構成一覧

| # | クラス / 関数 | 行付近 | 役割 |
|---|---------------|--------|------|
| — | `make_run_img_dir` | 58 | `img/<timestamp>/` 作成 |
| — | `_cholesky_lower` | 70 | Cholesky ラッパ |
| 1 | `Params` | 79 | 全パラメータ（PKG-A〜F、案B 含む） |
| 2 | `HexaModel` | 216 | 姿勢 CT モデル・Jacobian・Λ スケーリング |
| 2.5 | `PlantModel` | 355 | 11 状態プラント RK4 + PKG-E1〜E7 |
| 2.6 | `AltitudePID` | 514 | PID + 重力補償 + PKG-A bumpless |
| — | `t_cmd_to_u_hover` | 579 | T_cmd → per-motor duty（η 重み付き） |
| 2.7 | `DisturbanceManager` | 598 | 力/トルク外乱スケジュール |
| 2.8 | `NoiseModel` | 634 | ノイズ + PKG-E4 センサ遅延 + バイアス RW |
| 3 | `AdmmNmpcHex` | 743 | 凝縮 NMPC（PKG-C、案B、Cholesky リッジ fallback） |
| 4 | `FaultManager` | 1046 | η(t) + `apply_faults` |
| 5 | `Logger` | 1096 | 1 kHz ログ + 5×4 プロット + 水平 drift 注釈 |
| 6 | `sanity_checks` | 1334 | 起動時 24 項目（PKG-E 含む） |
| — | `compute_metrics` | 1729 | RMS / peak / sat / iter 統計 |
| 7 | `run_scenario` | 1766 | マルチレート + 全 PKG 統合ループ |
| 8 | `main` | 2159 | S1〜S8 一括実行 |

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

**プロパティ**: `u_hover_nom = m·g / (6·T_max) ≈ 0.120`

### 5.2 NMPC・マルチレート

| 変数 | 値 | 説明 |
|------|----|------|
| `N` | **30** | 予測ホライズン（旧 20 から延長） |
| `dt` | 0.01 s | NMPC 離散化周期（Sx/Su/H） |
| `dt_sim` | 0.001 s | プラント RK4 周期（1 kHz） |
| `nmpc_decimation` | 10 | NMPC 実効 100 Hz |
| `NX`, `NU` | 5, 6 | NMPC 状態・入力次元 |
| `NX_plant` | **11** | プラント状態次元（PKG-E5） |
| `NUT` | **180** | 凝縮 H 行列サイズ（NU·N = 6·30） |

### 5.3 高度 PID（PKG-B）

| 変数 | 値 | 説明 |
|------|----|------|
| `z_ref_default` | -2.0 m | 2 m ホバー目標（NED） |
| `Kp_z` | **2.0** | P ゲイン |
| `Ki_z` | 0.3 | 積分（定常偏差用） |
| `Kd_z` | **1.0** | D ゲイン（切替キック抑制） |
| `i_z_limit` | 0.2 N | 積分クランプ |
| `T_cmd_min_ratio` | 0.3 | 推力下限 = 0.3·mg |
| `trim_clip_cos` | 0.5 | cos(φ)cos(θ) 下限 |

### 5.4 Open-loop 離陸・Bumpless（PKG-A）

| 変数 | 値 | 説明 |
|------|----|------|
| `pwm_ascent` | **0.145** | 上昇 per-motor duty（PKG-E2/E3 マージン込み） |
| `z_switch_offset` | **1.0 m** | 目標より 1 m 手前で PID 切替 |
| `z_ramp_duration` | **3.0 s** | z_ref ランプ時間 |
| `u_ref_lpf_tau` | **0.0** | PKG-C 移行で LPF 無効 |
| `kd_fadein_duration` | 0.5 s | D 項 0→Kd_z フェードイン |
| `w_ref_ramp_duration` | 1.0 s | w_ref: w_at_restart→0 ランプ |

### 5.5 故障時 Bumpless（PKG-D）

| 変数 | 値 | 説明 |
|------|----|------|
| `fault_baseline_ramp_duration` | 0.3 s | u_baseline の故障時線形 ramp |
| `fault_iz_attenuation` | 0.5 | 故障時 PID 積分項減衰 |

### 5.6 コスト・RTI

| 変数 | 値 | 説明 |
|------|----|------|
| `Q_phi/Q_theta` | 100 | 姿勢角 |
| `Q_p/Q_q` | **10** | 角速度（旧 1 から増加） |
| `Q_r` | **0.5** | ヨー角速度（旧 1 から低減） |
| `R_u` | **14.0** | 入力ペナルティ（均一、案B でモータ別に上書き） |
| `terminal_scale` | 5 | 終端 Q 倍率 |
| `admm_max_iter` | 1 | ADMM 1 反復（実時間向け） |
| `reset_period` | 100 | フル再構築間隔 [NMPC 周期] |
| `eta_r_floor` | 0.1 | 案B: rbar = R_u / max(η, floor) |

### 5.7 ノイズ（真値/観測分離）

| 変数 | 値 | 説明 |
|------|----|------|
| `noise_enable` | True | False で perfect feedback |
| `sigma_phi/theta_meas` | 0.005 rad | IMU 姿勢 |
| `sigma_gyro_meas` | 0.010 rad/s | ジャイロ |
| `sigma_z/w_meas` | 0.020 / 0.050 | 高度・速度 |
| `sigma_p/q/r/w_proc` | 0.05 / 0.10 | プロセスノイズ |
| `sigma_gyro_bias_rw` | **0.0** | ジャイロバイアス RW（>0 で有効） |

### 5.8 PKG-E（実機リアリティ）— 個別無効化可能

| PKG | 変数 | デフォルト | 0/OFF で無効 |
|-----|------|-----------|-------------|
| E1 | `tau_motor` | 0.05 s | 0 |
| E1 | `actuator_delay_steps` | 2 | 0 |
| E2 | `plant_Ix_ratio` 等 | 1.20, 0.85, 1.10, 1.05, 0.95 | すべて 1.0 |
| E3 | `w_induced_hover` | 12.0 m/s | 0 |
| E3 | `aero_factor_min/max` | 0.30 / 1.20 | — |
| E4 | `imu_delay_steps` | 5 | 0 |
| E4 | `alt_delay_steps` | 20 | 0 |
| E6 | `J_prop_eq` | 5.0e-5 | 0 |
| E7 | `ge_height` | 0.5 m | 0 |
| E7 | `ge_max_boost` | 0.15 | — |
| E8 | `esc_bits` | 12 | 0 |

---

## 6. HexaModel（姿勢ダイナミクス）— NMPC 内部モデル

**変更なし（PKG-E の対象外）**。FDI 既知時のみ `set_eta(η)` で `B_eff = B_nom · diag(η)` を更新。

### 6.1 連続時間モデル

状態 **x = [φ, θ, p, q, r]**。`|v|ε = √(v² + ε²)` で C² 化。

### 6.2 B_c 行列（5×6）

```
B_c[2,k] = −y_k · T_max / Ix     (ロール)
B_c[3,k] = +x_k · T_max / Iy     (ピッチ)
B_c[4,k] = s_k · C_QT · T_max / Iz  (ヨー)
```

### 6.3 Λ スケーリングと box 制約

- `B_eff = B_nom · diag(η)` で FDI 既知時に NMPC 予測モデルを更新
- **u_max の η 比例縮減は廃止**（二重スケーリング防止）
- η≈0（完全故障）のみ `u_max[k] = 0`

---

## 7. PlantModel（11 状態プラント）— PKG-E

### 7.1 状態ベクトル（PKG-E5）

**x_plant = [φ, θ, p, q, r, z, w, x_pos, y_pos, u_vel, v_vel]**（11 次元）

| idx | 変数 | 意味 |
|-----|------|------|
| 0-4 | φ,θ,p,q,r | 姿勢（`HexaModel` on `p_plant`） |
| 5-6 | z, w | NED 鉛直位置・速度 |
| 7-8 | x_pos, y_pos | NED 水平位置 [m] |
| 9-10 | u_vel, v_vel | NED 水平速度 [m/s] |

7 次元入力は自動で 11 次元にパディング（後方互換）。

### 7.2 PKG-E2: プラント vs NMPC パラメータ不一致

```python
p_plant = copy.copy(p)
p_plant.Ix  = p.Ix  * plant_Ix_ratio   # 1.20 (+20%)
p_plant.Iy  = p.Iy  * plant_Iy_ratio   # 0.85 (-15%)
p_plant.Iz  = p.Iz  * plant_Iz_ratio   # 1.10 (+10%)
p_plant.mass = p.mass * plant_mass_ratio  # 1.05
p_plant.T_max = p.T_max * plant_T_max_ratio  # 0.95
self.attitude = HexaModel(p_plant)  # NMPC は p を使用
```

### 7.3 推力計算（PKG-E3 + E7）

```
T_total_raw = T_max_plant · Σ u
aero_factor = clip(1 − V_z / w_h, aero_factor_min, aero_factor_max)
ge_factor   = 1 + ge_max_boost · (1 − alt/ge_height)   # alt < ge_height
T_total = T_total_raw · aero_factor · ge_factor
```

### 7.4 並進ダイナミクス（NED, yaw=0）

```
ż = w
ẇ = g − T_total·cos(φ)·cos(θ)/m + F_z/m
ẋ_pos = u_vel,  ẏ_pos = v_vel
u̇_vel = −(T/m)·sin(θ) + F_x/m
v̇_vel = +(T/m)·sin(φ)·cos(θ) + F_y/m
```

`F_world[0,1]` は水平外力として **PKG-E5 でプラントに反映**（姿勢傾斜と合成）。

### 7.5 PKG-E6: プロペラジャイロ（簡易版）

```
H_prop = J_prop_eq · Σ(s_i · u_i)
τ_gyro_x = −H_prop · q,  τ_gyro_y = +H_prop · p
```

健全 6 機ホバーでは `Σ s_i·u_i = 0` → ジャイロ効果なし。1 機停止で対称性が崩れ非ゼロ。

### 7.6 PKG-E1: モータ動力学

```python
u_actual = plant.apply_actuator_dynamics(u_cmd_q, dt_sim)
# 1) FIFO 遅延 (actuator_delay_steps=2)
# 2) 一次遅れ: u_actual += α·(u_target − u_actual), α = dt/(τ+dt)
u_applied = η ⊙ u_actual   # 故障は u_actual に適用
```

---

## 8. PKG-C: 加算合成制御

### 8.1 制御構造

```
u_baseline = t_cmd_to_u_hover(T_cmd, η_control)   # PKG-F: FDI 未知は η_control=1
Δu         = nmpc.step(x_meas[:5], ..., u_baseline=u_baseline)
u_cmd      = clip(u_baseline + Δu, 0, 1)
```

NMPC の box 制約は動的更新:

```
Δu ∈ [U_min − u_baseline[k],  U_max − u_baseline[k]]   # 生存機（FDI 既知時）
Δu = 0                                                    # 故障機 (η_control≈0)
FDI 未知: 全機 box 開放（η_control=1、プラントのみ真 η）
```

`U_ref = 0`（差分のホバー基準はゼロ）。

### 8.2 U_ref LPF

`u_ref_lpf_tau = 0.0` で **無効**（`alpha_lpf=1` で通過）。構造は残存。

---

## 9. PKG-A / PKG-D: Bumpless Transfer

### 9.1 ascent → control（PKG-A）

切替時:
- `alt_pid.reset()`、`w_at_switch` 保存
- `nmpc.U, z, lam = 0`（Δu warm-start）
- `t_bumpless_restart = t_switch`

control 中（毎 NMPC 周期）:
```python
kd_scale = clip((t − t_restart) / kd_fadein_duration, 0, 1)
w_ref    = w_at_restart · max(0, 1 − (t − t_restart) / w_ref_ramp_duration)
z_ref    = 線形補間 z_at_restart → z_target  (z_ramp_duration=3 s)
```

### 9.2 故障時（PKG-D）

| 項目 | 動作 |
|------|------|
| D1 | `u_baseline`: 故障前値 → 新ホバー値を 0.3 s で ramp |
| D3 | `alt_pid.i_z *= 0.5` |
| D4 | `t_bumpless_restart` 再設定 → kd_scale / w_ref / z_ref ランプ再起動 |
| FDI 既知 | `set_eta`, `set_box`, `set_rbar_from_eta`, `force_reset` |

---

## 10. PKG-F: FDI 未知時の制御層分離

```python
eta_control = fault_mgr.eta.copy() if fdi_known else np.ones(6)
```

| 層 | FDI 既知 | FDI 未知 |
|----|----------|----------|
| プラント | 真 `η` | 真 `η` |
| `t_cmd_to_u_hover` | `η_control = η` | 全機健全として配分 |
| NMPC box | 故障機 Δu=0 | 全機 `[−u_base, 1−u_base]` |
| `B_eff` / 案B rbar | 故障反映 | 更新しない（均一 R_u） |

未知 FDI では NMPC が故障を「知らない」ため、生存機への推力再配分は **高度 PID のみ** が担う。

---

## 11. 案B: モータ別入力コスト

```python
def set_rbar_from_eta(self, eta):
    r_motor = R_u / max(η_k, eta_r_floor)   # 故障ほど Δu ペナルティ大
    # ホライズン全体にタイル
```

FDI 既知で故障イベント発生時に `set_rbar_from_eta` を呼び、部分故障機の過剰な Δu 振動を抑制する。

---

## 12. AltitudePID

### 12.1 制御則（NED）

```
e_z = z − z_ref
T_trim = m·g / max(cos(φ)·cos(θ), 0.5)
T_fb   = Kp·e_z + Ki·∫e_z + Kd·kd_scale·(w − w_ref)
T_cmd  = max(T_trim + T_fb, 0.3·m·g)
```

`kd_scale` は PKG-A/D の bumpless 用（0→1 フェードイン）。

### 12.2 t_cmd_to_u_hover

生存機（η > 1e-3）に `T_cmd` を均等配分: `per = T_cmd / (Ση · T_max)`。故障機は duty=0。

---

## 13. NoiseModel（PKG-E4）

### 13.1 真値/観測分離

- `noise_enable=False`: 真値そのまま（遅延も無効）
- `noise_enable=True`: 遅延バッファ → ノイズ追加

### 13.2 センサ遅延

| バッファ | 遅延 | 対象 idx |
|----------|------|----------|
| `_imu_delay` | 5 ms | 0..4 (φ,θ,p,q,r) |
| `_alt_delay` | 20 ms | 5,6 (z,w) |

### 13.3 プロセスノイズ・バイアス

RK4 積分後に `σ·√(dt_sim)` で p,q,r,w に加算。`sigma_gyro_bias_rw > 0` でジャイロバイアスを RW 更新。

---

## 14. AdmmNmpcHex（NMPC コア）

`sample/nmpc.cpp` の Python 鏡像。決定変数 `U ∈ R^180`（NU·N = 6·30）。

### 14.1 PKG-C 加算合成モード

- `u_baseline` 指定時: ロールアウト・Jacobian は `u_actual = Δu + u_baseline` で評価
- 返り値は **Δu**。`u_cmd = u_baseline + Δu`

### 14.2 Cholesky リッジ fallback

加算合成で H の条件数が悪化した場合、対角に `1e-3 … 10` を段階追加して Cholesky を再試行。

### 14.3 step() フロー

```
warm-start shift → Sx/Su 更新 (TR1/SR2 or フル再構築)
→ 勾配 g → ADMM 1 iter → Δu = z[:NU]
```

---

## 15. run_scenario（メインループ）

### 15.1 1 ステップの処理順（各 dt_sim）

```
1. FaultManager.update → η, events
2. DisturbanceManager.get_wrench → F_world, τ_body
3. η_control = η (FDI 既知) or 1 (未知)          ← PKG-F
4. フェーズ遷移 (ascent→control, fault 適用)
5. FDI ハンドラ (PKG-D + set_eta/box/rbar/reset)
6. 姿勢参照 x_ref、z_ref ランプ (PKG-A/D)
7. [100 Hz] PID → u_baseline、NMPC → Δu、u_cmd = u_base + Δu
8. ascent: u_cmd = pwm_ascent（NMPC 無視）
9. PKG-E8: u_cmd 量子化
10. PKG-E1: apply_actuator_dynamics → u_actual
11. u_applied = η ⊙ u_actual
12. PlantModel.step_rk4 (11 次元) → noise.apply_process → noise.observe
13. Logger.log
```

### 15.2 初期化

```python
x7_true = zeros(11)   # 地上
plant.reset_motors(pwm_ascent)  # PKG-E1
```

---

## 16. Logger と可視化

### 16.1 記録フィールド

| フィールド | 形状 | 内容 |
|------------|------|------|
| `x` | (T, ≥7) | プラント真値（11 次元格納可） |
| `x_meas` | (T, ≥7) | 観測値 |
| `u_cmd`, `u_app` | (T, 6) | コマンド / 実適用（η⊙u_actual） |
| `eta` | (T, 6) | モータ健全度 |
| `T_cmd`, `T_trim`, `T_fb` | (T,) | PID 内訳 |
| `F_d`, `tau_d` | (T, 3) | 外乱 |

### 16.2 プロット（5×4）+ PKG-E5 注釈

水平移動がある場合、タイトルに `horizontal drift: final=(x,y) m, max=...` を追記。

### 16.3 メトリクス出力

`compute_metrics` は `mean_iter_us`, `p50_iter_us`, `p99_iter_us` を返す。`scenario_metrics.csv` に保存。

---

## 17. sanity_checks（24 項目）

| # | テスト | 内容 |
|---|--------|------|
| 1-6 | 既存 | B_c、M1/M5 単独、ホバー、ランク、set_eta |
| 7 | PID | T_cmd = mg |
| 8-10 | PID | 重力補償、アンチワインドアップ |
| 11 | Plant | 姿勢・w_dot ≈ 0（p_plant ホバー duty 使用） |
| 12-13 | 外乱・u_hover | DisturbanceManager、t_cmd_to_u_hover |
| 14-15 | PKG-C | Δu≈0、u_total∈[0,1] |
| 16 | PKG-D1 | u_baseline ramp 補間 |
| **17** | **PKG-E1** | 一次遅れ τ 後 u_actual ≈ 63% |
| **18** | **PKG-E2** | p_plant ≠ p_nmpc |
| **19** | **PKG-E3** | 上昇中 w_dot 増大 |
| **20** | **PKG-E4** | IMU 遅延ステップ検出 |
| **21** | **PKG-E5** | 傾斜で水平加速度発生 |
| **22** | **PKG-E6** | ホバー時 Σs·u=0 |
| **23** | **PKG-E7** | 地面効果で w_dot 低下 |
| **24** | **PKG-E8** | 12-bit 量子化誤差 |

---

## 18. シナリオ一覧（main）

### 18.1 共通タイムライン

```
t = 0          地上、ascent（PWM=0.145/motor）
t ≈ 1.2 s      高度 1 m で PID 切替（z_switch = -1.0 m）
t ≈ 3.2 s      故障注入（切替+2.0 s）
t ≈ 8.2 s      phi_ref=0.0 rad（故障+5.0 s、同期イベント）
t = 15 s       終了
```

`PLAN_PHI_STEP = (5.0, 0.0)` — 姿勢参照は常に 0 rad（旧版の 0.2 rad ロールステップは廃止）。

### 18.2 各シナリオ

| ID | 故障 | FDI | 外乱 |
|----|------|-----|------|
| **S1** | なし | — | なし |
| **S2a** | M5 η=0.5 | 既知 | なし |
| **S2b** | M5 η=0.5 | 未知 | なし |
| **S3a** | M5 η=0（完全） | 既知 | なし |
| **S3b** | M5 η=0 | 未知 | なし |
| **S4** | **M5+M6 η=0** | 既知 | なし |
| **S6** | なし | — | F_z=+0.5 N, t∈[6,7) |
| **S7** | なし | — | τ_x=+0.02 N·m, t∈[6,7) |
| **S8** | M5 η=0 | 既知 | τ_x=+0.02 N·m, t∈[8,9) |

> **S4 変更**: 旧版の M3+M5 同時完全故障から **M5+M6（前右＋後左）** に変更。ヨー・ピッチの冗長性は残り、S4 は健全機に近い性能となる。

---

## 19. 実行結果（N=30, PKG-A〜F 適用後）

**参照実行**: `img/20260602_144130/`（`scenario_metrics.csv`）

### 19.1 サマリ表

| シナリオ | rms_φ [rad] | rms_θ [rad] | rms_z [m] | peak\|p\| | peak\|q\| | peak\|r\| | peak\|w\| [m/s] | sat [%] | mean iter [μs] |
|---------|-------------|-------------|-----------|-----------|-----------|-----------|------------------|---------|----------------|
| S1  | **0.003** | 0.007 | 0.479 | 0.05 | 0.15 | 0.11 | 1.11 | 0.0 | 2962 |
| S2a | 0.031 | 0.034 | 0.489 | 0.44 | 0.58 | 1.42 | 1.11 | 0.4 | 3221 |
| S2b | 0.038 | 0.039 | 0.506 | 0.40 | 0.39 | 1.43 | 1.11 | 0.7 | 1563 |
| S3a | 0.046 | 0.043 | 0.536 | 0.12 | 0.15 | 1.73 | 1.11 | 13.1 | 3381 |
| S3b | 0.102 | 0.053 | 0.644 | 0.63 | 0.56 | 2.99 | 1.11 | 4.9 | 1543 |
| S4  | **0.004** | 0.007 | 0.482 | 0.06 | 0.15 | 0.18 | 1.11 | 26.2 | 1500 |
| S6  | 0.003 | 0.007 | 0.485 | 0.05 | 0.15 | 0.11 | 1.11 | 0.0 | 1652 |
| S7  | 0.012 | 0.008 | 0.479 | 0.17 | 0.15 | 0.10 | 1.11 | 0.0 | 1513 |
| S8  | 0.051 | 0.042 | 0.539 | 0.23 | 0.20 | 1.76 | 1.11 | 13.3 | 3291 |

### 19.2 N=20 → N=30 の影響

| 項目 | N=20（旧レポート） | N=30（現行） |
|------|-------------------|-------------|
| H サイズ | 120×120 | **180×180** |
| mean iter (S1) | ~1050 μs | **~1900–3000 μs** |
| 実効 NMPC 周波数 | ~950 Hz 余裕 | **~270–550 Hz 余裕**（100 Hz 目標は依然達成） |
| 姿勢 RMS (S1) | 0.022 rad | **0.003 rad**（Q_p/q 増加の効果） |

### 19.3 主な発見

1. **健全機 S1**: rms_φ=0.003 rad、sat=0%。N=30 + コスト再調整で姿勢追従が大幅改善。
2. **部分故障 S2a/b**: FDI 既知/未知とも rms_φ<0.04 rad。案B + PKG-C が有効。
3. **完全故障 S3a（FDI 既知）**: rms_φ=0.05 rad、sat≈13%。1 機完全停止でも制御可能。
4. **S3b（FDI 未知）**: rms_φ=0.10 rad。PKG-F により NMPC は故障を補償せず、PID のみが推力再配分。
5. **S4（M5+M6）**: 旧 M3+M5 とは異なり **ほぼ健全並み**（rms_φ≈0.004）。sat は M5/M6 故障による一時的飽和で ~26%。
6. **外乱 S6/S7**: 健全時と同等の高度・姿勢性能。
7. **S8**: 故障+τ_x 重畳で sat≈13%、姿勢は許容内。
8. **計算時間**: mean 1.5–3.4 ms → 実効 **270–670 Hz**（目標 100 Hz の約 3–7 倍余裕）。

### 19.4 PKG-E 各要素の切り分け

全 PKG-E を OFF にするには:

```python
params.tau_motor = 0.0
params.actuator_delay_steps = 0
params.plant_Ix_ratio = params.plant_Iy_ratio = params.plant_Iz_ratio = 1.0
params.plant_mass_ratio = params.plant_T_max_ratio = 1.0
params.w_induced_hover = 0.0
params.imu_delay_steps = params.alt_delay_steps = 0
params.J_prop_eq = 0.0
params.ge_height = 0.0
params.esc_bits = 0
```

---

## 20. 依存関係と実行

```bash
cd program/NMPC_simulation

# サニティチェックのみ
python -c "from nmpc_simulation import Params, sanity_checks; sanity_checks(Params())"

# 全シナリオ実行
python nmpc_simulation.py
```

**依存**: NumPy, Matplotlib のみ（SciPy 不使用）  
**出力**: `img/<YYYYMMDD_HHMMSS>/sim_s*.png`（8 枚）+ `scenario_comparison.png` + `scenario_metrics.csv`

---

## 21. 設計上の制限と今後の拡張

| 項目 | 現状 | 備考 |
|------|------|------|
| yaw (ψ) | 未モデル化 | 水平移動は x,y のみ（PKG-E5） |
| 風場 | step/pulse のみ | 連続風は未実装 |
| NMPC モデル誤差 | プラント側のみ（E2） | NMPC 内部は公称値固定 |
| FDI 未知 | PKG-F で制御層のみ分離 | 推定器は未実装 |
| Pico 2 移植 | 未着手 | float64、H=180×180、N=30 で負荷増 |
| `nmpc_simulation_quad.py` | NU=4、同一 PKG | 冗長性 1 次元 |

---

## 22. 改訂履歴

| 日付 | 版 | 主な変更内容 |
|------|-----|-------------|
| 2026-05-25 | v1–v3 | マルチレート、Λ スケーリング、Open-loop 離陸、PID 再チューニング |
| 2026-05-26 | v5–v6 | シナリオ比較・**PKG-A/B/C/D** |
| 2026-05-27 | v7 | **PKG-E1〜E8**、プラント 11 次元、`pwm_ascent=0.145` |
| 2026-06-02 | **v8（最新）** | **N=30**、Q_p/q/r 再調整、**PKG-F**（FDI 未知）、**案B** `set_rbar_from_eta`、S4→M5+M6、`PLAN_PHI_STEP=(5,0)`、Cholesky リッジ fallback、ジャイロバイアス RW オプション、実行結果更新 |

---

*本レポートは `nmpc_simulation.py`（2322 行）のソース構造に基づく技術ドキュメントです。クアッド版は `nmpc_simulation_quad.py` を参照してください。Pico 2 移植時は `Params` と `sample/nmpc_params.hpp` を突き合わせてください。*
