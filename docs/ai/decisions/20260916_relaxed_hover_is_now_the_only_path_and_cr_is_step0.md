# relaxed hover は「やるべきか」から「これしか無い」に変わった。律速は `Cr` の同定

- **日付**: 2026-09-16
- **種別**: design | investigation（2026-09-15 計画書の改訂）
- **対象**: `docs/20260915_relaxed_hover_implementation_plan.md`, `nmpc_params.hpp`(`Cr`), `control.cpp`(x_ref / failsafe / EKF)
- **タグ**: #relaxed_hover #FTC #Cr #推力余裕 #EKF #遠心力 #FS_RATE_LIMIT #実装計画 #改訂

## 背景

`0916_100Hz_FTC_v1` で静的 4 発 FTC が 2 回とも姿勢を保てず壁に衝突。
実飛行較正した推力で見ると、担当 4 発の **0.95 までの余地は 0.011 しかない**
（[`20260916_bench_thrust_curve_overstates_flight_thrust_ftc_has_no_authority.md`](20260916_bench_thrust_curve_overstates_flight_thrust_ftc_has_no_authority.md)）。
この状況で relaxed hover の実装順序を再検討した。

## 判断根拠

### (1) 再計算：relaxed hover は余地を 19 倍に戻す

実飛行較正（`fmax = 0.7052 N`, `m = 285.3 g`）。
**揚力 = `mg` を満たす平衡のうち「最大 duty を最小化」する点**を探索した
（`ẋ = 0` の 5 式 + 揚力 1 式、box `0 ≤ f ≤ fmax`、コーニング解 `p=−Ω sinθ, q=Ω sinφ cosθ, r=Ω cosφ cosθ`）。

| 構成 | Ω | φ / θ | 最大 duty | **余地** |
|---|---|---|---|---|
| 健全 6 発 | 0 | 0 | 0.551 | 0.399 |
| **静的 4 発（現行）** | 0 | 0 | **0.939** | **0.011** |
| **relaxed hover（`Cr=3.7e-4`）** | **400 deg/s** | −10° / −10° | **0.742** | **0.208** |
| **relaxed hover（`Cr=9.5e-5`）** | **775 deg/s** | −4° / −6° | **0.738** | **0.212** |
| relaxed hover（`Cr=0.005` ファーム値） | 0 | 0 | 0.939 | 0.011 |

**健全機の 52% まで戻る。**

### (2) ⚠ 2026-09-15 の「軽量化のほうが得」は撤回

当時は 335 g / ベンチ曲線で「軽量化 +9 ポイント、relaxed hover は工数に見合わない」と判断した。
**軽量化は既に実施済み（335 → 285.3 g）で、それでも余地は 0.011。** 他に道が無い。

### (3) ⚠ 最重要 — ファームの `Cr = 0.005` のままでは relaxed hover は 1 ポイントも生まない

上表最下行。**モデルが「回しても得しない」と言うので、solver は回さない。**
`Cr` が大きいほど、回転を維持するのに必要なヨートルクが `Cr·Ω²` で増え、配分を食い潰す。
旨みは `Cr` が小さいときにしか出ない。実測は 9.5e-5 〜 3.7e-4 で、**ファーム値は 13〜53 倍過大**。

### (4) `Cr` の 4 倍のばらつきが、実現可能かどうかの境目を跨いでいる

| `Cr` | 必要 Ω | EKF 遠心誤差（IMU 10 mm / 20 mm オフセット） |
|---|---|---|
| 3.7e-4 | **400 deg/s** | 2.8° / 5.7° — 許容できる |
| 9.5e-5 | **775 deg/s** | 10.6° / 20.4° — ⚠ 姿勢推定が壊れる |

**`Cr` の同定が Step 0 になった。**

## 対応

計画書 `docs/20260915_relaxed_hover_implementation_plan.md` の冒頭に改訂章を追加し、
第 0 章（判断材料）を「無効」と明記。実施順序を改訂:

| # | 内容 |
|---|---|
| **0** | **`Cr` を確定**（`BENCH_YAW_EXCITE` の再実行。吊るすだけ・数分。実装済み） |
| **1** | `nmpc_params.hpp` の `Cr` を実測値へ。⚠ これ無しでは以降すべて無意味 |
| 2 | 平衡表の生成 + ホバーからの到達可能性シミュ |
| 3 | `control.cpp:2332` の `x_ref`（`p_ref`,`q_ref` が初めて非零）と `U_ref = u_eq − u_hover` |
| 4 | `FS_RATE_LIMIT`（`control.cpp:201`, 現行 360 deg/s）を `Ftc_mode` 中のみ緩和 |
| 5 | EKF の遠心補正 `a − ω×(ω×r_imu)` |
| 6 | 段階投入: プロペラ無し → 治具固定 → 係留 → 自由飛行 |

### 却下した案

- **`Cr` を測らずに実測範囲の中間値を入れる**: (4) のとおり範囲の両端で「実現可能／EKF が壊れる」が
  分かれる。中間値を入れても、どちらに転ぶか分からないまま実機に進むことになる
- **軽量化を続けて静的 4 発で成立させる**: 余地 500 deg/s² 相当を得るのに **追加 −24 g** が必要
  （[`20260916_bench_thrust_curve...md`](20260916_bench_thrust_curve_overstates_flight_thrust_ftc_has_no_authority.md)）。
  プロペラガードを外した後にさらに 24 g は現実的でない
- **`Q_p` を上げて姿勢を安定させる**: 余地 0.011 では何を指令しても出ない。重み以前の問題

## 検証結果

- 平衡探索は `model_f()` の式そのまま（ジャイロ項 `(Iz−Iy)qr` 等 + 2 乗ダンピング）を使用
- 配分は「A f = M_req, Σf = mg/(cosφcosθ), 0 ≤ f ≤ fmax」の 1 次元残差を三分探索で最小ピーク化
- ⚠ **推力は実飛行較正したが、モーメントは実推力 `T(u)` で計算した。ファームのモデルは
  モーメントを duty に比例（`T_max·u`）としており、ここに既知の不一致がある。** 未解消
- ⚠ `Cp`, `Cq` は未測定の仮値 0.025 のまま。平衡はこれに依存する
- ⚠ **到達可能性（ホバーから平衡へ行けるか）は未評価。** 計画書 Step 2 の主目的
- ⚠ 机上のみ。実機・シミュとも未実施

## 次に触る人へ

- ⚠ **`Cr` を直す前に Step 2 以降へ進まないこと。** モデルが平衡の存在を否定している状態で
  参照だけ入れても、solver は `R_u` に引かれて静的解へ戻る
  （[`20260915_uref_must_point_at_the_balanced_allocation.md`](20260915_uref_must_point_at_the_balanced_allocation.md) と同じ失敗の形）
- ⚠ `FS_RATE_LIMIT = 360 deg/s` は平衡回転数（400〜775）より低い。**Step 4 は必須で、Step 3 と同時に入れる**
- **relaxed hover は上昇能力をほとんど持たない**。高度を上げられない飛行になることを実験計画に織り込む
- `ftc = 1` の実機ログは 0916 で初めて 2 本取れた。プロペラ無し・治具固定の段階は今すぐできる
- 関連: [`20260905_relaxed_hover_eval_cr_decides.md`](20260905_relaxed_hover_eval_cr_decides.md)（当時から「Cr が決める」と書いてあった）、
  [`20260915_relaxed_hover_plan_equilibrium_is_representable_ekf_is_the_blocker.md`](20260915_relaxed_hover_plan_equilibrium_is_representable_ekf_is_the_blocker.md)、
  [`20260914_stage12_even_torque_confirmed_and_first_cr_estimate.md`](20260914_stage12_even_torque_confirmed_and_first_cr_estimate.md)
