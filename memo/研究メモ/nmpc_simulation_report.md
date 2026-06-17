# nmpc_simulation.py 理解レポート

> Hexa-X ヘキサコプタの 4 入力 NMPC を 6 入力に拡張し、ADMM ベース耐故障姿勢制御を Python シミュレータ上で検証するプログラム

---

## 目次

1. [概要](#1-概要)
2. [システム全体像](#2-システム全体像)
3. [クラス構成と責務](#3-クラス構成と責務)
4. [数式と実装の対応](#4-数式と実装の対応)
5. [1 制御周期の処理フロー](#5-1-制御周期の処理フロー)
6. [シミュレーション設計](#6-シミュレーション設計)
7. [Logger と評価メトリクス](#7-logger-と評価メトリクス)
8. [C++ 実装との対応 (Pico 2 移植視点)](#8-c-実装との対応-pico-2-移植視点)
9. [既知の制限・実装ノート](#9-既知の制限実装ノート)
10. [動かし方](#10-動かし方)
11. [参考文献](#11-参考文献)

---

## 1. 概要

### 1.1 目的

- 既存の **クアッドコプタ用 ADMM-NMPC (C++, Raspberry Pi Pico 2 ターゲット)** を、**ヘキサコプタ X 配置 (6入力)** に拡張し、Python シミュレータで動作検証する。
- **モータ故障時の耐故障制御** (Fault-Tolerant Control, FTC) を、完全停止 / 部分故障 / FDI 既知/未知 などの条件で比較評価する。
- C++ 実装の Python 鏡像として作り、後の Pico 2 ポーティング時に挙動の参照点として使えるようにする。

### 1.2 スコープ

| 含む | 含まない |
|---|---|
| 姿勢 (φ, θ, p, q, r) 制御 | 位置 (x, y, z) 制御 |
| 6入力 NMPC ソルバ (ADMM + SR2) | アロケーション行列 (NMPC が直接 6 duty を最適化) |
| RK4 プラント / Euler 内部モデル | 風外乱・センサノイズ |
| モータ故障 (η=0, 0.5 等) | FDI ロジック (オラクル故障時刻を与える) |

### 1.3 主要な設計判断

- **機体配置**: ArduPilot 標準 Hexa-X (90°, -90°, -30°, 150°, 30°, -150°)、回転方向 CW/CCW 交互
- **状態**: `[φ, θ, p, q, r]` (NX=5)
- **入力**: `[u_M1, ..., u_M6]` の duty ∈ [0, 1] (NU=6)
- **コスト**: Bryson's rule (Q_φ=Q_θ=100, Q_p=Q_q=Q_r=1, R_u=17, terminal ×5)
- **R_u = 17** はクアッド版 25 を `25 × 4/6` で再スケール
- **ADMM**: ρ=1.0, 1 iter (RTI)
- **SR2**: Powell damped BFGS 相当、リセット周期 100、故障検知時は即時リセット
- **MPM 実験**: プラントは RK4、NMPC 内部は Euler
- **故障処理**: U_max 縮減 + U_ref(ホバー duty) シフト + SR2 強制リセット

---

## 2. システム全体像

### 2.1 ブロック図

```
                                  ┌──────────────────────────────┐
   ref_schedule ────────────────► │     run_scenario (loop)      │
   fault_schedule ───────────────►│                              │
                                  │  ┌────────────────────────┐  │
                                  │  │ FaultManager.update(t) │  │
                                  │  └──────────┬─────────────┘  │
                                  │             ▼                │
                                  │  [if event & FDI_known]      │
                                  │    set_box(u_max[k]=η)       │
                                  │    update U_ref (hover shift)│
                                  │    force_reset()             │
                                  │             │                │
                                  │             ▼                │
                                  │  ┌────────────────────────┐  │
                          x ────► │  │   AdmmNmpcHex.step()   │  │
                                  │  │  - SR2 rebuild or upd. │  │
                                  │  │  - gradient            │  │
                                  │  │  - ADMM 1 iter         │  │
                                  │  └──────────┬─────────────┘  │
                                  │             │ u_cmd          │
                                  │             ▼                │
                                  │      η · u_cmd = u_app       │
                                  │             │                │
                                  │             ▼                │
                                  │  ┌────────────────────────┐  │
                                  │  │ HexaModel.step_rk4()   │──┼─► x_{t+1}
                                  │  │   (真のプラント)       │  │
                                  │  └────────────────────────┘  │
                                  │             │                │
                                  │             ▼                │
                                  │       Logger.log(...)        │
                                  └──────────────┬───────────────┘
                                                 ▼
                                          plot / metrics
```

### 2.2 信号の流れ (1 制御周期)

```
t  ──► FaultManager ──► η(t), events
t  ──► ref_schedule ──► x_ref (5,)
                                         │
                          ┌──────────────┘
                          ▼
            x (現状態) ──► NMPC ──► u_cmd ∈ [0,1]^6
                                         │
                                         ▼
                          u_applied = η ⊙ u_cmd (要素積)
                                         │
                                         ▼
                          x_next = RK4(x, u_applied, dt)
```

---

## 3. クラス構成と責務

| クラス | ファイル位置 | 責務 |
|---|---|---|
| `Params` (dataclass) | 74-126 | 物理・NMPC・コスト・RTI 定数を一元管理。`nmpc_params.hpp` の Python 版 |
| `HexaModel` | 131-252 | 連続時間ダイナミクス `f(x,u)`、ヤコビアン `(A_c, B_c)`、離散化 (Euler/RK4) |
| `AdmmNmpcHex` | 258-489 | NMPC ソルバ本体。凝縮形 QP + SR2 ヘシアン更新 + ADMM 1-iter + warm-start |
| `FaultManager` | 495-524 | スケジュール式に η(t) を返すオラクル。FDI イベントを発火 |
| `Logger` | 530-666 | 時系列ログの蓄積 + 4×4 サブプロット可視化 |
| `sanity_checks` (func) | 672-714 | B_c の符号・rank・ホバートリムの自動検証 |
| `run_scenario` (func) | 757-835 | 1 シナリオの実行ループ |
| `main` (func) | 841-934 | S1-S4 を順次実行 + サマリ表 |

---

## 4. 数式と実装の対応

### 4.1 ヘキサコプタダイナミクス

連続時間 5 状態ダイナミクス:

$$
\begin{aligned}
\dot\phi &= p + q\sin\phi\tan\theta + r\cos\phi\tan\theta \\
\dot\theta &= q\cos\phi - r\sin\phi \\
\dot p &= \frac{\tau_x - (I_z - I_y)qr - C_p\, p\,|p|}{I_x} \\
\dot q &= \frac{\tau_y - (I_x - I_z)rp - C_q\, q\,|q|}{I_y} \\
\dot r &= \frac{\tau_z - (I_y - I_x)pq - C_r\, r\,|r|}{I_z}
\end{aligned}
$$

`|·|` は数値安定化のため `sqrt(v² + ε²)` で平滑化 (ε = 1e-3)。

実装: `HexaModel.f()` (174-195行)、`_abs_s()` で平滑化。

### 4.2 入力ヤコビアン B_c (5×6)

モータ位置 $(x_i, y_i)$、回転符号 $s_i \in \{-1, +1\}$ から:

$$
B_c[2, i] = \frac{-y_i \cdot T_{\max}}{I_x},\quad
B_c[3, i] = \frac{+x_i \cdot T_{\max}}{I_y},\quad
B_c[4, i] = \frac{s_i \cdot C_{QT} \cdot T_{\max}}{I_z}
$$

実装: `HexaModel._build_Bc()` (158-165行)。

| モータ | 角度 [deg] | (x_i/L, y_i/L) | 回転 | s_i |
|---|---:|---|---|---:|
| M1 右横 | 90 | (0, +1) | CW | −1 |
| M2 左横 | −90 | (0, −1) | CCW | +1 |
| M3 前左 | −30 | (+√3/2, −1/2) | CW | −1 |
| M4 後右 | 150 | (−√3/2, +1/2) | CCW | +1 |
| M5 前右 | 30 | (+√3/2, +1/2) | CCW | +1 |
| M6 後左 | −150 | (−√3/2, −1/2) | CW | −1 |

`rank(B_c[2:, :]) = 3` (3 制御自由度 = ロール/ピッチ/ヨー) で、6 入力との差 3 が冗長性 → null space が ADMM box 制約と組み合わさり最小ノルム解を自然に与える。

### 4.3 凝縮形 QP

ホライズン全体の状態軌道を入力で表す:

$$
X = S_x x_0 + S_u U
$$

ここで $X \in \mathbb{R}^{NX \cdot N}$, $U \in \mathbb{R}^{NU \cdot N}$、$S_x, S_u$ は離散化線形モデル $A_d, B_d$ から構築。

コスト関数:

$$
J = (X - X_{\text{ref}})^\top \bar Q (X - X_{\text{ref}}) + (U - U_{\text{ref}})^\top \bar R (U - U_{\text{ref}})
$$

$\bar Q, \bar R$ は対角ブロック (最終ステージのみ `terminal_scale = 5` 倍)。

入力で勾配を取ると:

$$
\nabla_U J = S_u^\top \bar Q (S_x x_0 - X_{\text{ref}}) + \bar R (U - U_{\text{ref}}) \\
H = S_u^\top \bar Q S_u + \bar R
$$

実装:
- `_full_rebuild_sx_su()` (331-353行) — $S_x, S_u$ の再帰構築
- `_full_rebuild_chol()` (356-366行) — $H$ の構築と Cholesky
- `_compute_gradient()` (411-415行) — 勾配計算

### 4.4 TR1 (Broyden rank-1) で Sx, Su を差分更新

毎周期フル再構築 ($N \cdot N$ 行列積) は重いので、ヤコビアンの「効果」だけを rank-1 で補正:

$$
\begin{aligned}
s_x &= x_0^{\text{now}} - x_0^{\text{prev}} \\
s_u &= U^{\text{now}} - U^{\text{prev}} \\
r &= X^{\text{NL}}_{\text{now}} - (S_x s_x + S_u s_u + X^{\text{lin}}_{\text{prev}}) \\
u_{\text{rk1}} &= r / (\|s_x\|^2 + \|s_u\|^2) \\
S_x &\leftarrow S_x + u_{\text{rk1}} \otimes s_x \\
S_u &\leftarrow S_u + u_{\text{rk1}} \otimes s_u
\end{aligned}
$$

実装: `_tr1_update()` (373-383行)。`secant_min_norm = 1e-6` 以下なら更新スキップ。

### 4.5 SR2 (rank-2) で H を更新

TR1 で $S_u$ が rank-1 変化したとき、$H = S_u^\top \bar Q S_u + \bar R$ も rank-2 で変化する:

$$
\begin{aligned}
a &= S_u^\top (\bar Q u_{\text{rk1}}) - v_u \cdot c, \quad c = u_{\text{rk1}}^\top \bar Q u_{\text{rk1}} \\
p &= a + \tfrac{1}{2}(c+1) v_u \\
q &= a + \tfrac{1}{2}(c-1) v_u \\
H &\leftarrow H + p p^\top - q q^\top
\end{aligned}
$$

実装: `_sr2_update()` (386-408行)。`|c| > 1e6` は数値発散と判定して `need_full_rebuild` を立てる。

> **Python 実装上の制約**: C++ では Eigen の `LLT.rankUpdate(±1)` を 2 回当てれば $O(n^2)$ で済むが、SciPy/NumPy に同等関数がないため、$H$ を直接更新して全 Cholesky を再計算 ($O(n^3)$)。Python シミュ専用と割り切る。

### 4.6 ADMM 1 iteration (box 制約)

最適化問題:

$$
\min_U\; \tfrac{1}{2}U^\top H U + g^\top U \quad \text{s.t.}\quad u_{\min} \le U \le u_{\max}
$$

ADMM 分割 $U = z$ で:

$$
\begin{aligned}
U^{k+1} &= (H + \rho I)^{-1} (-g + \rho(z^k - \lambda^k)) \\
z^{k+1} &= \mathrm{clip}(U^{k+1} + \lambda^k,\; u_{\min},\; u_{\max}) \\
\lambda^{k+1} &= \lambda^k + U^{k+1} - z^{k+1}
\end{aligned}
$$

RTI なので 1 反復のみ。実装: `_admm()` (418-433行)。

> $H + \rho I$ の `+ρI` は `_full_rebuild_chol()` の時点で `H` の対角に既に加算済み (360行)。box の上限 `hi` は `_box_lo_hi()` でモータ毎の `u_max_per_motor` を N 個並べた `(NU·N,)` ベクトル。

### 4.7 故障処理 (ハイブリッド) — 実装は U_max + U_ref シフト

レポートの§3で議論した3方式のうち、**U_max 縮減** + **U_ref ホバー再分配** + **SR2 強制リセット** を実装。

```python
# run_scenario 783-803行 抜粋
if events and fdi_known:
    for k_fail, eta_t in events:
        # ① box 制約縮減: u_max[k] = η
        new_u_max = nmpc.u_max_per_motor.copy()
        new_u_max[k_fail] = float(eta_t)
        nmpc.set_box(nmpc.u_min_per_motor, new_u_max)

        # ② u_hover を生存機側に再分配
        alive = fault_mgr.eta > 1e-3
        n_alive = int(np.sum(alive))
        per = params.mass * params.gravity / (n_alive * params.T_max)
        u_hover = np.where(alive, per, 0.0)
        U_ref = np.tile(u_hover, params.N)

        # ③ SR2 履歴強制リセット (B が急変するため古い曲率は有害)
        nmpc.force_reset()
```

完全停止 (η=0) では `u_max[k]=0` で M_k が事実上ゼロ固定されるため十分。
**部分故障 (η=0.5) では B_eff = η · B_c が未実装**のため、NMPC は依然「M_k は満効率」と思っている。詳細は §9.1 参照。

---

## 5. 1 制御周期の処理フロー

`AdmmNmpcHex.step()` (446-489行) の流れ:

```
入力: x0 (現状態, 5), X_ref (NXT,), U_ref (NUT,)
出力: u_opt (NU,)  ← 先頭ステージの最適入力

[1] warm-start シフト
    U, z, λ を 1 ステージ前にずらす (末尾はホールド)

[2] Sx/Su 構築
    if force_reset or cycles_since_reset >= 100:
        Sx, Su をフル再構築           ← O(N²) 行列積
        H を再構築 + Cholesky          ← O(NUT³)
        cycles_since_reset = 0
        last_was_reset = True
    else:
        X_nl ← 非線形ロールアウト
        TR1 rank-1 更新                ← O(NXT)
        SR2 rank-2 で H 更新           ← O(NUT²) (Pythonは再Choleskyで O(NUT³))
        cycles_since_reset += 1

[3] 勾配計算
    g = Su^T Q̄ (Sx x0 - X_ref) - R̄ U_ref

[4] ADMM 1 iter
    rhs = -g + ρ(z - λ)
    U = H^{-1} rhs              ← np.linalg.solve
    z = clip(U + λ, lo, hi)
    λ = λ + U - z

[5] 結果取り出し
    u_opt = clip(z[:NU], u_min_per_motor, u_max_per_motor)

[6] 次周期用スナップショット保存
    x0_prev = x0
    U_lin_prev = U_lin
    X_lin_prev = Sx x0 + Su U_lin
```

### 計算複雑度の目安

| 演算 | 規模 (NU=6, N=20) | 概算 FLOP |
|---|---|---:|
| 非線形ロールアウト | N × NX × NU | ~2,400 |
| Sx, Su フル再構築 | N² × NX × NU | ~14,400 |
| H 構築 (Su^T Q̄ Su) | NXT × NUT² = 100 × 120² | ~1,440,000 |
| Cholesky (NUT × NUT) | NUT³/3 = 120³/3 | ~576,000 |
| TR1 + SR2 更新 (Python版) | NUT² + 再Cholesky | ~720,000 |
| ADMM 1 iter (solve + clip) | NUT² + NUT | ~14,400 |

Python (NumPy) で **mean ≈ 1-3 ms / iter** が期待値 (実機 Pico 2 で C++ + SR2 rank-update を使えば 300-500 μs に圧縮可能、レポート §5 参照)。

---

## 6. シミュレーション設計

### 6.1 モデルプラント不一致 (MPM)

| 役割 | 積分法 | 故障モデル |
|---|---|---|
| プラント (真) | RK4 (4次精度) | η · u_cmd が実機に入る |
| NMPC 内部モデル | Euler (1次精度) | FDI 既知なら u_max[k]=η, 未知ならそのまま |

これは意図的な不一致。実機でも NMPC は完全な真のダイナミクスを持たないため、シミュレータでこのギャップをモデル化するのが重要 (レポート §A.2)。

### 6.2 シナリオ一覧

| ID | 状況 | 故障 | FDI | t_end | ref |
|---|---|---|---|---|---|
| S1 | 健全ベースライン | なし | — | 2.5 s | φ: 0→0.2→0 |
| S2a | M5 部分故障 | η=0.5 @ t=2s | 既知 | 4.5 s | φ: 2 段階 |
| S2b | M5 部分故障 (passive) | η=0.5 @ t=2s | **未知** | 4.5 s | φ: 2 段階 |
| S3a | M5 完全停止 | η=0 @ t=2s | 既知 | 4.5 s | φ: 2 段階 |
| S3b | M5 完全停止 (passive) | η=0 @ t=2s | **未知** | 4.5 s | φ: 2 段階 |
| S4 | M5+M3 隣接2機故障 | η=0,0 @ t=2s | 既知 | 3.0 s | φ: 0→0.2→0 |

**S4 は制御不能を意図** (Hexa-X で前ペアが両方落ちるとヨー軸のヘルスが極端に崩れる)。失敗パターンを可視化する目的。

### 6.3 サニティチェック (`sanity_checks`, 672-714行)

| 項目 | 期待 | 失敗時の意味 |
|---|---|---|
| `B_c[2, 0]` = `-L·T_max/Ix` | 一致 | アロケーション式の符号エラー |
| M1 のみ duty=1 → ṗ < 0 | 右翼上昇 → 左ロール | y_M1 の符号反転 |
| M5 のみ duty=1 → q̇ > 0 かつ ṗ < 0 | 前右モータ | x_M5 / y_M5 の符号 |
| 全機 u_hover → 角加速度 ≈ 0 | ホバートリム | 機体パラメータ不整合 |
| 総推力 = m·g | 一致 | T_max or u_hover_nom 計算ミス |
| `rank(B_c[2:, :])` = 3 | 完全姿勢制御可能 | アロケーション degenerate |

`main()` の最初に必ず実行。失敗すると `AssertionError` で停止し、後続シナリオは走らない。

---

## 7. Logger と評価メトリクス

### 7.1 ログ項目 (`Logger.log()`, 541-549行)

| 名前 | 形 | 内容 |
|---|---|---|
| `t` | float | 時刻 [s] |
| `x` | (5,) | 状態 [φ, θ, p, q, r] |
| `u_cmd` | (6,) | NMPC 出力 (制約適用後) |
| `u_app` | (6,) | プラントに入った値 = η ⊙ u_cmd |
| `eta` | (6,) | モータ健全度 |
| `iter_us` | float | NMPC.step() の時間 [μs] |
| `was_reset` | bool | この周期でフル再構築したか |
| `x_ref` | (5,) | 目標状態 |

### 7.2 メトリクス (`compute_metrics()`, 739-754行)

| 名前 | 定義 | 用途 |
|---|---|---|
| `rms_phi` / `rms_theta` | √mean((x - x_ref)²) | 追従精度 |
| `peak_p` / `peak_q` / `peak_r` | max\|·\| | 過渡時の振り回し度合い |
| `sat_rate` | 入力が 0 or 1 になっている時間比率 | 飽和度 |
| `mean_iter_us` / `p50` / `p99` | NMPC iter 時間統計 | リアルタイム性 |

### 7.3 4×4 サブプロット (`Logger.plot()`)

| 行/列 | (0,0) | (0,1) | (0,2) | (0,3) |
|---|---|---|---|---|
| **0** | φ vs ref | θ vs ref | 姿勢誤差 | NMPC iter [μs] |
| **1** | p [rad/s] | q [rad/s] | r [rad/s] | SR2 reset フラグ |
| **2** | u_cmd M1/M2 | u_cmd M3/M4 | u_cmd M5/M6 | Σ u_applied |
| **3** | u_app M1-M3 | u_app M4-M6 | η_i 全6機 | 累積飽和率 |

PNG は `img/<YYYYMMDD_HHMMSS>/sim_*.png` に保存される (run 単位でフォルダ分離)。

---

## 8. C++ 実装との対応 (Pico 2 移植視点)

### 8.1 ファイル対応

| C++ | Python | 備考 |
|---|---|---|
| `nmpc_params.hpp` | `Params` dataclass (74-126) | `NU=4 → 6`、`R_u=25 → 17` |
| `nmpc_model.cpp` の `model_f` | `HexaModel.f` (174-195) | xdot 式は同型、B_c が 4列→6列 |
| `nmpc_model.cpp` の `model_B_const` | `HexaModel._build_Bc` (158-165) | ハードコード行列 → 動的構築 |
| `nmpc_model.cpp` の `model_jacobians` | `HexaModel.jacobians` (197-234) | A_c は同型 |
| `nmpc.cpp` の凝縮ステップ | `AdmmNmpcHex._full_rebuild_sx_su` (331-353) | アルゴリズム同一 |
| `nmpc.cpp` の TR1 | `_tr1_update` (373-383) | 同一 |
| `nmpc.cpp` の SR2 | `_sr2_update` (386-408) | C++: rankUpdate, Py: 再 Cholesky |
| `nmpc.cpp` の ADMM | `_admm` (418-433) | 同一 |
| `control.cpp` の制御ループ | `run_scenario` (757-835) | より単純化、FDI はオラクル |

### 8.2 移植時の主要変更点

1. **`B_c` を 5×6 に拡張** — `nmpc_model.cpp` の `model_B_const` を本 Python 版と同じく M1-M6 の角度・回転テーブルから構築するように書き換え
2. **`StateVec`/`InputVec` のサイズ変更** — テンプレート or `constexpr` で NU=6 に
3. **モータ毎の `u_max_per_motor`** を C++ に追加 — 現状は `U_max` グローバルなのを配列化
4. **FDI 連動の SR2 強制リセット API** — `control.cpp` から `nmpc.force_reset()` 相当を呼べるように
5. **U_ref (ホバー duty) の動的更新** — `control.cpp` 側でモータ alive 数から計算しソルバに渡す

### 8.3 Pico 2 上の期待性能

レポート §5 の試算より:
- NMPC iter ≈ 50,000 FLOP/周期 ≈ **330 μs @ 150 MHz**
- 100 Hz (10 ms) で **CPU 使用率 3〜10%**
- Python シミュ (~1-3 ms) の 5-10 倍高速化される見込み

---

## 9. 既知の制限・実装ノート

### 9.1 部分故障時の B_eff スケーリング未実装 ⚠

レポート §3 で議論したハイブリッド方式 (Λ + U_max) のうち、現状は U_max のみ。

**問題**: η=0.5 の部分故障で NMPC は依然 B_c (満効率版) を使うため、

```
NMPC 予測: u_k=0.5 → トルク = B_c[:,k] × 0.5  = 0.5·B
実プラント: u_k=0.5 → トルク = η · B_c[:,k] × 0.5 = 0.25·B
```

予測誤差が 2 倍積もる。完全停止 (η=0, u_max=0 で固定) では問題なし。

**追加すべき処理 (run_scenario 内, 例)**:

```python
# 現状: u_max のみ
new_u_max[k_fail] = float(eta_t)

# 追加: B_eff スケーリング (NMPC モデルに反映)
model.Bc[:, k_fail] = eta_t * Bc_nominal[:, k_fail]
# ※ もしくは AdmmNmpcHex 側に scaled-B を保持
```

ただし `Bc` を書き換えると次の `jacobians()` 呼び出しでも反映される必要があり、`_build_Bc()` を呼び直すか別の `Bc_eff` を保持する設計変更が要る。**S2a vs S2b の差が想定より小さい場合に追加すべき**。

### 9.2 SciPy の rank-update 関数欠如

C++ 版は Eigen の `LLT.rankUpdate(v, ±1)` で SR2 を $O(n^2)$ で更新。Python では `H` を更新して `np.linalg.cholesky` で再因数分解 ($O(n^3)$)。シミュレーション速度に影響するが結果は同一。

### 9.3 オラクル FDI

`FaultManager` は故障時刻と motor index を**事前に与える**。実機の FDI (オブザーバ残差 + 閾値判定) は未実装。レポート §B 参照。

### 9.4 姿勢制御専用

z 軸の高さ制御がない。`Σ u_app ≈ m·g/T_max` を NMPC は直接保証しないため、高度ループは別途必要。総推力サニティは Logger の `Σ u_applied` プロットで目視。

### 9.5 隣接 2 機故障 (S4) は失敗想定

Hexa-X で前ペア (M3+M5) が両方落ちると、

- M3, M5 はどちらも CW=−1, CCW=+1 の混合で前方寄り
- 残る 4 機 (M1,M2,M4,M6) でロール・ピッチ・ヨー全部を作るのは厳しい
- 特にピッチ軸が degenerate (前方の x_i > 0 が消失)

**意図的にコントローラビリティ境界を見せるシナリオ**。失敗が観察できなければシナリオ設定を見直す。

### 9.6 単精度 vs 倍精度

Python は全て `float64`。Pico 2 は単精度 FPU。SR2 / ADMM の数値感度の差は別途検証が必要 (レポート §Phase 4)。

---

## 10. 動かし方

### 10.1 必要環境

```
Python 3.9+
NumPy, Matplotlib  (SciPy は不要)
```

### 10.2 実行

```bash
cd /path/to/scripts
python3 nmpc_simulation.py
```

### 10.3 出力

- **コンソール**: 各シナリオの NMPC iter 時間、追従誤差、サチュ率、最後にサマリ表
- **PNG**: `img/YYYYMMDD_HHMMSS/sim_s{1,2a,2b,3a,3b,4}_*.png` (6 ファイル)
- **GUI**: Qt5Agg/TkAgg が使えれば最後に `plt.show()` でウィンドウ表示

### 10.4 確認すべきこと

1. `=== Sanity Checks ===` が `→ all sanity checks PASSED.` で終わるか
2. S1 (健全) の `rms_phi` が 0.01 rad 程度に収まっているか
3. S3a vs S3b で **既知 (S3a) の方が整定が速い** か
4. S4 で姿勢が発散していること (意図通りの失敗)
5. `mean_iter_us` が 1000-3000 μs 程度か (Python NumPy の典型値)

---

## 11. 参考文献

レポート本体に詳細あり。本実装で特に参照したもの:

- **アロケーション**: ArduPilot `AP_MotorsHexa.cpp`, PX4 `MultirotorMixer/geometries`
- **故障モデル**: Falconí & Holzapfel, *AMCS* 28(2):237-249, 2018; Sensors PMC 6873231
- **ADMM box prox**: Parikh & Boyd, "Proximal Algorithms" §6.2
- **TinyMPC**: Alavilli/Nguyen et al., arXiv:2310.16985 (Cortex-M7 ベンチ)
- **準ニュートン (SR2/BFGS)**: CMU stat lecture 17 (Powell damping)
- **NMPC FTC**: Nan/Scaramuzza, IEEE RA-L 2022 (ACADO+RTI クアッド); Tzoumanikas et al., IFAC 2018 (ヘキサ 3 機停止)
