# 実装計画書: 3 軸アクチュエータ一次遅れを持つ NX=8 / N=12 姿勢 NMPC

- **作成日**: 2026-09-20
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/`
- **最終構成**: `x = [φ, θ, p, q, r, a_p, a_q, a_r]`, `NX=8`, `NU=6`, `N=12`
- **関連資料**:
  - [`20260920_chatgpt_handoff.md`](20260920_chatgpt_handoff.md)
  - [`../program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/docs/motor_lag_sysid_analysis.md`](../program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/docs/motor_lag_sysid_analysis.md)
  - [`ai/decisions/20260910_delay_compensation_new_solver.md`](ai/decisions/20260910_delay_compensation_new_solver.md)
  - [`ai/decisions/20260910_v10_delay_comp_removed_4hz_ekf_lean_remains.md`](ai/decisions/20260910_v10_delay_comp_removed_4hz_ekf_lean_remains.md)
  - [`ai/decisions/20260920_tmax_mapping_and_curve_shape_closed_plant_gain_retracted.md`](ai/decisions/20260920_tmax_mapping_and_curve_shape_closed_plant_gain_retracted.md)

---

## 0. 目的と完了条件

### 目的

現行モデルはモータ指令 `Δu` が同じ周期内に角加速度へ直接作用すると仮定し、別経路の
`delay_comp_steps=6` が直近 duty で状態を 60 ms 前方予測している。しかし 0919_v8 では
DC6 が有効かつ `pred_nonfinite=0` だったにもかかわらず、2–3 Hz の発散へ移行した。

本変更では、モータ・ESC・プロペラ系の一次遅れを NMPC の予測状態に明示し、

1. 指令が角加速度へ即時に作用しないこと
2. 飽和解除後も実効モーメントが残ること
3. 周波数に比例して位相を進め続ける DC6 と、一次遅れの周波数特性との差

を予測へ反映する。

### 完了条件（DoD）

#### 数式・モデル

1. 状態が `x=[φ,θ,p,q,r,a_p,a_q,a_r]` の 8 次元である。
2. 3 軸遅れ状態が、同一時定数を持つ 6 モータ一次遅れモデルと単精度誤差内で一致する。
3. 離散遷移と解析ヤコビアンが有限差分に対して相対誤差 `1e-3` 以下で一致する。
4. 健全時に `B_att·1=0`、`rank(B_att)=3` が起動時 sanity check に合格する。
5. `Q_a_p=Q_a_q=Q_a_r=0` とし、初版では遅れ状態そのものを追従コストで罰しない。

#### 計算資源

6. `NX=8`, `N=12`, `NX_TOTAL=96`, `NU_TOTAL=72` がログヘッダと ELF で確認できる。
7. `EIGEN_NO_MALLOC` を維持し、制御ループ内の動的確保がない。
8. Pico 2 W 実測で通常経路の `iter_us p95 ≤ 8.5 ms`、周期全体の `loop_us p95 ≤ 10 ms`。
9. フル再構築の中央値が現行 28.3 ms を大幅に超えない。暫定上限は 32 ms、最大 35 ms。
10. BSS は現行から `+8 KiB` 以内を目標とし、スタック・ヒープを除く SRAM 余裕を 80 KiB 以上残す。

#### 制御性能

11. 現行 `NX=5,N=14,DC6` と比べ、0.2–1 Hz の姿勢追従を 20% 以上悪化させない。
12. プラント `τ=35–60 ms`、純遅延 0–2 周期、慣性 `±15%` の格子で発散しない。
13. 2–5 Hz の角速度ピークと 5 Hz 超のモーメント指令を現行以下にする。
14. ADMM 非有限、双対リセット、`U_lin` ガードの発火を現行健全便より増やさない。
15. DC6 と一次遅れを二重適用しない。初期値は `delay_comp_steps=0` とする。

#### 運用

16. デサチュレーション・400 Hz clamp・フェイルセーフを通過した**実際の出力 duty**で
    アクチュエータ状態推定を更新する。
17. 健全飛行を確認するまで貨物便・FTC 便へ進まない。
18. 実測前の計算時間や安定性を「確認済み」と書かない。

---

## 1. 設計判断

### 1.1 6 モータ状態ではなく、3 軸角加速度状態へ厳密縮約する

各モータの正規化実効推力を `ω∈R⁶`、指令 duty を `u∈R⁶` とする。
全モータの時定数 `τ` が同じなら、

```text
ω_dot = (u - ω) / τ
```

姿勢へ作用するモータ由来の角加速度を

```text
a = B_att ω  ∈ R³
```

と置ける。したがって、

```text
a_dot = B_att ω_dot
      = (B_att u - a) / τ
```

となる。姿勢 5 状態に `a_p,a_q,a_r` を加えるだけで、姿勢軸については 6 モータ状態と等価になる。

採用する連続時間モデル:

```text
φ_dot, θ_dot : 現行のオイラー角運動学

p_dot = a_p + (-(Iz-Iy) q r - Cp p|p|) / Ix
q_dot = a_q + (-(Ix-Iz) r p - Cq q|q|) / Iy
r_dot = a_r + (-(Iy-Ix) p q - Cr r|r|) / Iz

[a_p,a_q,a_r]_dot = (B_att u - [a_p,a_q,a_r]) / τ
```

`B_att` は現行 `model_B_const()` の行 2–4 と同じであり、単位は
`rad/s² per duty`。新しいゲイン `K` は追加せず `K=1` とする。
推力ゲインは従来どおり `T_max` と `C_QT` が担う。

#### この縮約の成立条件

- 6 モータの時定数を同一とみなせる。
- 現行どおり duty→推力を線形近似する。
- 高度・共通推力を予測状態に含めない。
- 個別モータ RPM の推定を目的としない。

モータ個体差、電圧依存、実測推力曲線の凹性はこの初版ではモデル化しない。

### 1.2 FTC の `η` を `B_att` へ重ねて掛けない

現行 FTC は `η` を box 上限

```text
Δu_j ∈ [-u_hover, η_j U_max - u_hover]
```

へ反映し、故障発の絶対 duty を 0 に固定する。モデル入力 `Δu` に対しても幾何学的に
`B_att·1=0` なので、

```text
B_att Δu = B_att (u_abs - u_hover·1) = B_att u_abs
```

が成立する。したがって `B_att` の列へさらに `η` を掛けると、部分故障を二重計上する恐れがある。
初版では `η` は従来どおり box のみに使う。

故障切替直後の残留推力は、実出力 duty で更新される `a_hat` が時定数 `τ` で減衰させる。

### 1.3 離散化はアクチュエータ部分を厳密 ZOH にする

`dt=0.01 s`、`α=exp(-dt/τ)` とし、

```text
β = τ(1-α)
γ = dt-β
```

を定義する。1 周期中の duty を一定とすれば、

```text
a_{k+1} = α a_k + (1-α) B_att u_k

rate_{k+1}
  = rate_k
  + dt·rigid_nonlinear(x_k)
  + β a_k
  + γ B_att u_k
```

となる。角度運動学と剛体非線形項は現行どおり 1 次離散化する。

これにより、単純 Euler の `dt/τ=0.2` でも生じる時定数誤差と、
「入力が次周期まで角速度へ全く効かない」という余分な 1 サンプル遅れを避ける。

`model_jacobians()` は連続時間ヤコビアンを作り、`discretize_euler()` で
アクチュエータ関連ブロックだけ上記の厳密係数へ上書きする。
`simulate_step()` も同じ式を使い、フル再構築と非線形 rollout の離散モデルを一致させる。

### 1.4 遅れ状態のコストは初版では 0

`a` は「目標角加速度」ではなく、未観測の内部アクチュエータ状態である。
初版で `a→0` を直接罰すると、新しい減衰ノブを同時に導入して原因分離できなくなる。

```text
Q = diag(Qφ,Qθ,Qp,Qq,Qr,0,0,0)
Qf = terminal_scale·diag(Qφ,Qθ,Qp,Qq,Qr,0,0,0)
```

とする。`R_u`、既存 Q、終端倍率は変更しない。
将来 `Q_a` を導入する場合は別の tuning decision とする。

### 1.5 DC6 は初期状態で無効化する

現行の 40–50 ms はモータ単体の時定数ではなく、計算・ZOH・モータを含む「実効遅れ」の可能性がある。
`τ=45 ms` と DC6=60 ms を同時に入れると二重補償になる。

初期実装:

```text
tau_act = 0.045 s   // 実装・ホスト検証用の暫定値。物理同定済みとは扱わない
delay_comp_steps = 0
```

ホスト格子で必要性が確認された場合のみ、計算・ZOH の純遅延分として
`delay_comp_steps=1`、最大でも 2 を比較する。6 へ戻さない。

### 1.6 N=12 は NX=8 と同じコミットで入れない

最終構成は `NX=8,N=12` だが、原因分離のため実装順は次とする。

1. 現行 `NX=5,N=14,DC6` の基準値を保存
2. `NX=8,N=14,DC0` でモデル変更単体を検証
3. `NX=8,N=12,DC0` でホライズン短縮の差分を測定

通電するのは最終構成のみとし、中間構成はホスト・Pico ベンチ計測専用とする。

---

## 2. 計算量・メモリ予測

### 2.1 固定次元

```text
現行: NX_TOTAL=5×14=70, NU_TOTAL=6×14=84
中間: NX_TOTAL=8×14=112, NU_TOTAL=84
最終: NX_TOTAL=8×12=96, NU_TOTAL=72
```

凝縮後の Hessian/LLT 次元は `NU_TOTAL` で決まるため、最終構成では
`84×84 → 72×72` へ小さくなる。

### 2.2 主要静的領域の概算

`Sx`, `Su`, `Su_w`, `M`, LLT 内部行列、固定長 scratch vector を数えた概算:

```text
NX=5,N=14 : 約110.5 KiB（基準）
NX=8,N=14 : 約141.7 KiB（+31.2 KiB）
NX=8,N=12 : 約105.7 KiB（基準比 -4.8 KiB）
```

これは `g_nmpc_ctx` 周辺の主要配列だけの概算であり、最終値は ELF map と
`arm-none-eabi-size` を正とする。

### 2.3 演算量の方向

- ADMM の前進後退代入: おおむね `(72/84)² = 0.735` 倍
- フル LLT: おおむね `(72/84)³ = 0.630` 倍
- Hessian 内積: `NX` 増加と `N` 短縮がほぼ相殺
- `Sx/Su` 伝播: 状態次元増加で増えるが、Hessian/LLT より小さい
- 通常周期の `Su^TQx`、TR1/SR2: 行数 `70→96` で増える

したがって `NX=8,N=12` は現行と同程度、または少し速い可能性があるが、**未実測**である。

---

## 3. 変更対象

### 3.1 `nmpc_params.hpp`

変更:

- `mpc::NX: 5 → 8`
- 最終段階で `mpc::N: 14 → 12`
- `physical::tau_act = 0.045f` を暫定追加
- `rti::delay_comp_steps: 6 → 0`
- `NX_TOTAL` / `NU_TOTAL` の古いコメントを 96 / 72 へ更新
- `sqrt(84)`、120 次元等の古い説明を現在値へ更新

変更しない:

- `Q_phi,Q_theta,Q_p,Q_q,Q_r,R_u`
- `admm_rho=1`
- `admm_max_iter=4`
- `tr1_c_max`
- `tr1_skip_max_run`
- `reset_period`

### 3.2 `nmpc_model.hpp`

追加:

```cpp
using AttAccelVec = Eigen::Matrix<float, 3, 1>;
using AttInputMat = Eigen::Matrix<float, 3, mpc::NU>;

void model_B_att(AttInputMat &B_att);
void model_attitude_accel_from_duty(const InputVec &u, AttAccelVec &a_cmd);
```

既存の `StateVec`, `StateMat`, `InputMat` は `mpc::NX` に追従する。

### 3.3 `nmpc_model.cpp`

実装項目:

1. 現行のモータ幾何から 3×6 の `B_att` を構築
2. `model_f()` を 8 状態へ拡張
3. `model_jacobians()` に以下を追加
   - `A_c(2:4,5:7)=I`
   - `A_c(5:7,5:7)=-I/τ`
   - `B_c(5:7,:)=B_att/τ`
   - `B_c(2:4,:)=0`
4. `discretize_euler()` のアクチュエータ関連ブロックを厳密 ZOH 係数へ上書き
5. `simulate_step()` を同じ離散式へ変更
6. `model_sanity()` を `B_att` に対する検査へ変更

注意:

- 現行 `bottomRows<3>()` は新状態ではアクチュエータ行を指すため使用禁止。
- `for (i=2; i<NX; ++i)` で `B·1` を検査する現行コードも意味が変わるため廃止する。
- `model_f()` と `simulate_step()` の入力は従来どおり `Δu` とする。

### 3.4 `nmpc.cpp`

変更:

- `build_cost_diag()` の Q/Qf を 8 要素化し、末尾 3 要素を 0 にする。
- `nmpc_delay_predict()` の健全性判定に `a_p,a_q,a_r` の有限性を追加する。
- `delay_comp_steps=0` の初期構成では predictor を呼ばないことを確認する。
- `nmpc_warmup()` は全 8 状態ゼロで開始する。
- 5/84/120 等の古いコメントだけを現行値へ直す。

初版では行わない:

- `NmpcCostParams` や Wi-Fi API への `Q_a` 追加
- zero-weight 行を省略する Hessian 専用最適化
- ADMM の再調整

計算時間が DoD を超えた場合に限り、`qbar==0` のアクチュエータ行を
Hessian/勾配計算から省略する最適化を第 2 段として検討する。

### 3.5 `nmpc.hpp`

- 状態定義コメントを 8 状態へ更新
- 凝縮次元コメントを N=12 に更新
- 新しい actuator state の単位を明記
- `NmpcContext` の構造自体は固定サイズ型経由で自動追従

### 3.6 `control.cpp`: 実出力に基づくアクチュエータ状態推定

#### 400 Hz 側

400 Hz 出力段で、フェイルセーフ、ゲート、故障モータ、minimum duty、
デサチュレーションをすべて適用した後の duty を `u_pwm` とする。

```text
α400 = exp(-0.0025/τ)
a_hat ← α400 a_hat + (1-α400) B_att u_pwm
```

を実行する。18 MAC/周期程度であり、400 Hz 側への負荷は小さい。

`motor_stop()` が選択された周期は `u_pwm=0` として更新し、状態を瞬時に 0 へリセットしない。
実モータも時定数を持って減速するためである。ただしディスアーム状態が十分続けば自然に 0 へ収束する。

#### core0→core1 受け渡し

3 float を途中状態で読まないよう、sequence counter とメモリバリアを持つ小さなスナップショットを使う。

```text
writer(core0): seqを奇数化 → 3要素書込 → barrier → seqを偶数化
reader(core1): 偶数seqを読取 → 3要素コピー → barrier → seq再確認
```

ロック待ちで 400 Hz 出力を止めない。

#### 100 Hz 側

NMPC 呼び出し前に、

```text
x0(0:4) = 現行 EKF/ジャイロ状態
x0(5:7) = a_hat snapshot
```

とする。

参照は、

```text
x_ref(0)=phi_ref
x_ref(1)=theta_ref
x_ref(2)=0
x_ref(3)=0
x_ref(4)=r_ref
x_ref(5:7)=0
```

とする。末尾 3 状態の Q は 0 なので `x_ref(5:7)` はコストへ影響しないが、
未初期化を避けるため必ず明示する。

#### relaxed hover

`Relax_x_eq[5]` のまま `for(i<NX)` を回すと範囲外アクセスになる。
初版は以下のどちらかに統一する。

- `Relax_x_eq[8]` に拡張し、末尾を 0 で初期化
- blend を最初の 5 状態だけに限定し、`x_ref(5:7)=0` を維持

推奨は `Relax_x_eq[8]`。将来 `Q_a` を導入する場合は
`a_eq=B_att·Relax_u_eq` を入れる。初版では `Q_a=0` なので末尾 0 で挙動は変わらない。

### 3.7 デサチュレーションとの接続

別タスクで実装する健全時デサチュレーションとの契約:

- 段 1 の共通 sag は線形モデル上 `B_att·1=0` なので `a_hat` を変えない。
- 段 2 の差分スケーリングは実効モーメントを変えるため、**必ずスケーリング後 duty**で `a_hat` を更新する。
- FTC 中はデサチュレーションを無効化する。
- ログへ `sag`, `alpha_desat`, `post_desat_mean` を残し、observer 入力と一致させる。

### 3.8 `control.hpp` / テレメトリ / ログ

100 Hz リングへ最初から 3 列追加すると長さが減るため、初版は以下を優先する。

- 起動ヘッダ: `NX=8 N=12 tau_act=... delay_comp=0`
- 10 Hz ベンチ/USB 診断: `a_hat_p,a_hat_q,a_hat_r`
- 100 Hz リング: 空きがなければ `a_hat_norm` 1 列のみ、または既存の低価値列と交換

飛行前のベンチでは 3 軸すべて取得する。列追加時は `DATANUM` と `g_log_cols_written` を必ず一致させる。

### 3.9 Python シミュレータ

`program/NMPC_simulation/nmpc_simulation.py` は現行 C++ パラメータと乖離しているため、
ファーム変更の正しさをこのファイルだけで判定しない。

実施順:

1. C++ と同じ `B_att`, `τ`, 8 状態離散式を小さな独立テストへ実装
2. 6 モータ `ω` モデルとの等価性を確認
3. その後 `HexaModel` / NMPC 側へ NX=8 を移植
4. C++ と Python の 1 step 出力を固定 fixture で照合

シミュレータのプラント側は 6 モータ状態を維持し、コントローラだけ 3 軸縮約状態にする。
同じ簡略モデル同士を比較して安定性を過大評価しないためである。

---

## 4. 実装順序とコミット単位

### Phase 0: 現行基準値の固定

コード変更前に同じ `build_fresh` で取得:

- `arm-none-eabi-size`
- map 上の `g_nmpc_ctx`, `Logdata`
- `model_sanity`
- ベンチ 60 s の通常 `iter_us/loop_us` 分布
- フル再構築 20 回以上の時間分布
- 静的姿勢誤差に対する `|Δu|` と率/角比
- `0919_v8` リプレイ時の nonfinite/reset/TR1 skip

成果物とビルド時刻を記録し、以後の比較基準にする。

### Phase 1: `B_att` 抽出とモデル単体テスト（NX=5 のまま）

変更:

- `model_B_att()`
- 6 モータ一次遅れと 3 軸縮約の等価性テスト
- exact ZOH 係数テスト

この段階では飛行ファームの状態次元と挙動を変えない。

コミット例:

```text
test(nmpc): 3軸アクチュエータ遅れの等価性テストを追加
```

### Phase 2: 実出力 observer と core 間 snapshot（まだ NX=5）

変更:

- core0 で最終 PWM duty をミラー
- 400 Hz `a_hat` 更新
- core1 snapshot
- ベンチログ

`a_hat` はまだ NMPC へ渡さず、観測性と計算時間だけ確認する。

コミット例:

```text
feat(nmpc): 最終PWM出力から3軸アクチュエータ状態を伝播
```

### Phase 3: NX=8 / N=14 / DC0

変更:

- モデル・ヤコビアン・cost・x0/x_ref を 8 状態化
- `delay_comp_steps=0`
- relaxed hover 配列修正
- N は 14 のまま

目的:

- 状態拡張単体の計算時間と制御差を測る
- DC6 との二重補償を排除

この構成は通電飛行せず、ホストと Pico ベンチだけで評価する。

コミット例:

```text
feat(nmpc): 3軸一次遅れ状態を予測モデルへ追加
```

### Phase 4: N=14 → 12

変更は `mpc::N` と関連コメントだけを基本とする。
重み、ADMM、TR1/SR2 しきい値は同時に変更しない。

コミット例:

```text
perf(nmpc): NX8の計算予算確保のため予測ホライズンを12へ短縮
```

### Phase 5: 計算最適化（DoD 未達時のみ）

優先順:

1. `qbar=0` の actuator 行を Hessian 重み付け・内積から除外
2. 3×6 `B_att` と actuator block の疎構造を手展開
3. 不要になった DC6 の 4 分割 RK2 コードを飛行ビルドから除外

最初から最適化を混ぜない。モデルの正しさと速度変更の原因を分ける。

### Phase 6: ドキュメント・decision

実測後に更新:

- `Benzen_NMPC/docs/NMPC_REPORT.md`
- `Benzen_NMPC/docs/motor_lag_sysid_analysis.md`
- `Benzen_NMPC/plan.md`
- `docs/benzen_nmpc_build.md`
- `docs/ai/repository_rules.md`
- `documents/論文/.../paper.typ`（実測時間が変わった場合）

---

## 5. 検証計画

### 5.1 モデル単体

#### T1: common mode

```text
B_att·[1,1,1,1,1,1]^T = [0,0,0]
```

許容誤差: 各軸 `1e-5 rad/s²` 以下。

#### T2: rank

`rank(B_att)=3`。

#### T3: 6 状態モデルとの等価性

乱数 duty、初期 `ω`、`τ={0.035,0.045,0.060}`、10 秒相当の系列で、

```text
a_6 = B_att ω
a_3 = 3軸縮約モデル
```

を比較する。float32 で最大誤差 `1e-4 rad/s²` 以下。

#### T4: 63.2% 応答

`B_att u` のステップに対し `t=τ` で最終値の 63.2%±1%。

#### T5: 解析ヤコビアン

各状態・入力へ中心差分を与え、離散 1 step の `A_d,B_d` と比較する。

- 通常状態
- `p,q,r=0`
- `p,q,r=±300 deg/s`
- duty 中央
- box 上限近傍

相対誤差 `1e-3` 以下。絶対値が小さい要素は絶対誤差 `1e-5` 以下。

#### T6: failure decay

1 発 duty を 0 へ落とし、6 モータモデルと 3 軸モデルの残留モーメント減衰が一致する。

### 5.2 ソルバ静的回帰

- `model_sanity=PASS`
- 全状態・全行列 finite
- `M` SPD、LLT 成功
- ADMM 4 反復
- `U_opt` が box 内
- `EIGEN_RUNTIME_NO_MALLOC` をホストで有効化して allocation 0
- 健全時 `u_hover` の common mode を変えても姿勢予測が変わらない
- `η=0` の故障発が絶対 duty 0 になる

### 5.3 ホスト閉ループ格子

コントローラモデル:

```text
NX8, N12, tau_model ∈ {0.035,0.045,0.060}
delay_comp_steps ∈ {0,1,2}
```

真のプラント:

```text
6モータ一次遅れ
tau_plant ∈ {0.035,0.040,0.050,0.060}
pure_delay ∈ {0,1,2} cycle
T_gain ∈ {0.8,1.0,1.2}
Ix,Iy ∈ {0.85,1.0,1.15} × nominal
```

最低シナリオ:

1. 水平ホバー 30 s＋センサ雑音
2. ロール ±5°、0.5 Hz
3. ピッチ ±5°、0.5 Hz
4. 5° step
5. 既存 0919_v8 の参照・外乱リプレイ
6. 上限近傍からの飽和解除
7. 健全時デサチュレーション段 1 / 段 2
8. 1 発 `η:1→0` 切替（飛行許可ではなく数値回帰）

評価:

- 発散・非有限 0
- 姿勢 90°、角速度 360 deg/s の安全限界へ到達しない
- 2–5 Hz / 5–15 Hz / 15–40 Hz の状態・モーメント指令エネルギー
- box 飽和率
- step 63%, 90%, overshoot
- `|Δu|` p50/p90/p99
- 現行 `NX5,N14,DC6` との差

### 5.4 Pico ベンチ

プロペラを外した状態で:

1. 60 s 実行を 5 回
2. 周期再構築を最低 20 回取得
3. `iter_us`, `loop_us`, path, reset cause を保存
4. `pred_nonfinite`, lambda reset, nonfinite, ulin reset を確認
5. `a_hat` の step 応答が `τ` と一致することをログ確認
6. `arm-none-eabi-size` と map を保存

比較は同じクロック・同じビルド種別・同じログ設定で行う。

### 5.5 実機飛行（実験可能になってから）

順序を固定する。

1. プロペラ付き地上・低スロットル
2. 係留または安全治具
3. 貨物なし・中立ホバー 5 s
4. 貨物なし・±3°単発入力
5. 貨物なし・10 s ホバー
6. 25 g を剛固定し中立ホバー
7. 貨物あり・±3°単発入力

大舵、ヨー指令、貨物振り子状態、FTC は上記が合格するまで禁止する。

---

## 6. 採用判定とロールバック

### 採用条件

以下をすべて満たす場合のみ標準構成へ昇格する。

- モデル単体 T1–T6 合格
- ホスト格子で現行より発散条件が増えない
- 5 Hz 超モーメント指令が現行以下
- Pico の 100 Hz期限を満たす
- メモリ余裕を満たす
- 貨物なし実機で DC6 構成と同等以上

### 却下・再設計条件

- `tau_model` の小変更で安定／発散が反転する
- `delay_comp_steps=0,1,2` のどれでも格子を通らない
- N=12 で静的剛性または追従が 20%以上悪化
- 通常周期 p95 が 10 msを超える
- フル再構築が 35 msを超える
- actuator observer とジャイロ応答の整合が便ごとに大きく変わる

### ロールバック

コミットを Phase ごとに分け、問題の層だけ revert できるようにする。

```text
Phase 4 revert: NX8,N14へ戻す
Phase 3 revert: NX5,N14,DC6へ戻す
Phase 2は観測のみなので残しても制御挙動不変
```

旧 UF2、旧 `#params/#delay_comp/#cost` ヘッダ、旧 map を必ず保存する。

---

## 7. 既知の限界

1. 閉ループ飛行ログから `τ` を点推定できていない。45 ms は暫定値である。
2. 電池電圧を測っておらず、時定数と定常ゲインの電圧依存を補正できない。
3. 実測推力曲線は凹だが、予測モデルは duty 線形のままである。
4. モータ個体差を 1 個の `τ` に縮約している。
5. observer は RPM/推力を観測せず、指令 duty だけで伝播する。
6. 共通推力・高度は状態に含まれない。
7. 構造振動・貨物振り子はこのモデルでは表現できない。

これらを理由に一次遅れモデルを「同定済み真値」とは扱わない。
本変更の主張は「未モデルだったアクチュエータ状態を、現時点の最良範囲で明示し、
ロバスト性格子と実測時間で採否を決められる構造にする」ことまでとする。

---

## 8. 実装時チェックリスト

- [x] 現行ベースラインを保存
- [x] `B_att·1=0`, rank=3
- [x] 6 モータモデルとの等価性
- [x] exact ZOH とヤコビアン有限差分
- [x] core0 の**最終出力 duty**で observer 更新
- [x] core 間 snapshot が途中値を読まない
- [x] `x0(5:7)` と `x_ref(5:7)` を必ず初期化
- [x] `Relax_x_eq` の範囲外アクセスを除去
- [x] `Q_a=0`
- [x] DC6 を無効化
- [x] NX8/N14 を先に測定（クロスビルドとmap。Pico timingは未測定）
- [x] N12 を別段階で適用
- [x] `sqrt(NU_TOTAL)` 依存箇所を確認
- [x] `DATANUM == written`
- [x] `EIGEN_NO_MALLOC`
- [x] fresh build
- [x] ELF size / map
- [ ] Pico 実測 timing
- [x] decision と INDEX
- [x] repository rulesを更新（論文は新しい実測値がないため保留）

---

## 9. 2026-09-20 実施結果

### 完了

- Phase 0: 既存飛行ログと旧 `build_fresh` mapを保存。旧構成の実機時間は履歴値として維持。
- Phase 1: `B_att`を独立API化し、T1–T6とソルバ静的回帰のホストテストを追加。
- Phase 2: 400 Hz最終PWM observerとsequence counter snapshotを実装。
- Phase 3: NX=8/N=14・DC0をクロスビルド。data+bssは463,672 B。
- Phase 4: NX=8/N=12へ変更し、freshクロスビルドとELF size確認まで完了。
- Phase 5: DoDのPico実測前なので追加最適化は実施しない（条件付きPhase）。
- Phase 6: NMPC_REPORT、motor lag分析、plan、build手順、repository rules、
  decision、INDEXを更新。論文は新しい実測値が無いため変更しない。

### 未検証（実装完了とは分離）

- 5.3の全閉ループ格子
- Pico上の100 Hz timing、60 sベンチ、`a_hat`実測応答
- プロペラなし通電、係留、自由飛行

最終N=12のfresh buildは成功し、`text=691,344 B`, `bss=417,720 B`。
ホストC++ T1–T6・solver no-mallocとPython sanityは全項目PASS。
実機試験は元々「実験可能になってから」の項目であり、
起動ヘッダとプロペラなしベンチに合格するまで飛行禁止とする。
