# 271 g で静的 4 発 FTC の姿勢権限が 7 倍になった。配分コードは 4 発対応済み。⚠ 昨日緩めたフェイルセーフを静的試験に戻した

- **日付**: 2026-09-16
- **種別**: investigation | bugfix
- **対象**: `Benzen_NMPC/control.cpp`(failsafe / `ekf_drift_watch`), `nmpc_params.hpp`(mass), FTC 配分経路の監査
- **タグ**: #FTC #4発 #軽量化 #推力余裕 #FS_RATE_LIMIT #EKF #Relax_enable #box天井

## 背景

**285.3 g → 271 g に軽量化**し、静的 4 発 FTC（FR 停止・対角 RL を 0）を再試行したい。
「現行の FTC プログラムが 4 発に対応しているか」の監査依頼。

## 判断根拠

### (1) ★14 g の軽量化で姿勢権限が 7 倍になった★

実飛行較正（健全便 `0916_100Hz_normal_v1` の静穏ホバー `u_base = 0.5506` @ 285.3 g、
スケール 0.8389、`f_max = T(0.95) = 0.7052 N`）で解き直した:

| 質量 | 健全ホバー duty | 必要/最大 | 担当発 duty | 上方向余地 | ロール権限 | ピッチ権限 |
|---|---|---|---|---|---|---|
| 0.3350 | 0.676 | 116.5% | **不可** | — | — | — |
| 0.3130 | 0.619 | 108.8% | **不可** | — | — | — |
| **0.2853** | 0.551 | 99.2% | 0.939 | **0.011** | **54** | **44** deg/s² |
| **0.2710** | 0.516 | **94.2%** | **0.874** | **0.076** | **390** | **314** deg/s² |
| 0.2600 | 0.490 | 90.4% | 0.826 | 0.124 | 639 | 514 |

**14 g 減っただけで余地が 0.011 → 0.076（7 倍）、ピッチ権限が 44 → 314 deg/s²。**
推力曲線が上端で寝ている（`T'(u)` が小さい）ため、質量差に対して duty 差が大きく出る。

⚠ ただし健全 6 発の 4474 deg/s² に対しては**まだ 9%**。余裕がある状態ではない。

### (2) 配分コードは 4 発対応済み（ホスト検査で確認）

`control.cpp` の FTC 経路に「健全 5 発」を前提にした箇所は**無い**
（`n_healthy` は旧実装を説明するコメントに残るのみ）。
`u_ref_shift` の carry 判定・400 Hz clamp をそのまま抜き出してホスト検査（`ftc4.cpp`）:

- FR 停止で `carry = {FL, MR, ML, RR}`、**`n_carry = 4`**
- `u_base` 0.55〜0.63 の全点で `Σk_Xm·u = Σk_Ym·u = Σk_S·u = 0`、FR と RL の参照は絶対 0
- 400 Hz clamp: 故障発 FR → 0、対角 RL は**下限 0.1 が当たらず指令値のまま**、担当 4 発には下限 0.1 が残る

**すべて PASS。コード側の修正は不要だった。**

### (3) ⚠ 昨日の変更が静的 4 発試験には不利だった（訂正）

[`20260916_ftc_gated_failsafe_and_relaxed_hover_reference_plumbing.md`](20260916_ftc_gated_failsafe_and_relaxed_hover_reference_plumbing.md)
で `FS_RATE_LIMIT` と `EKF_DRIFT_LIMIT` を **`Ftc_mode` だけを条件に**緩めた。
しかし relaxed hover を使わない**静的 4 発 FTC でも保護が 360 → 1200 deg/s に弱まる**。
静的 4 発では `r` は高々 90 deg/s 程度（`0916_100Hz_FTC_v1` 実測）なので、360 deg/s を残すのが正しい。

→ 条件を **`(Ftc_mode && Relax_enable)`** に狭めた。`Relax_enable` はソースを書き換えないと
1 にならないので、**既定では静的 FTC も健全時と同じ保護**に戻る。

### (4) ⚠ 使えるスロットル窓が狭い

担当発の参照は `1.5 × u_base` なので:

| | |
|---|---|
| 271 g のホバーに必要な `u_base` | **0.583**（= 0.874/1.5） |
| 参照が box を出る `u_base` | **0.633**（= 0.95/1.5） |

**窓は 0.583〜0.633 の 8% 幅しかない。**
⚠ `u_base > 0.633` では参照が box の外を指し、`R_u` が全発を天井へ押すので**姿勢権限が消える**。
スロットルを上げすぎることが、上げ足りないのと同じくらい危険。

## 対応

| 箇所 | 変更 |
|---|---|
| `control.cpp:1497` | `rate_lim` の条件を `Ftc_mode` → `(Ftc_mode && Relax_enable)` |
| `control.cpp:1317` | `drift_lim` も同様 |
| ログヘッダ | `rate_dps_ftc` → `rate_dps_relax`、`drift_lim_ftc_dps` → `drift_lim_relax_dps`（実態に合わせた名前） |
| `nmpc_params.hpp:41` | `mass` は**ユーザが既に 0.2713 へ更新済み**。履歴コメントのみ整理 |

### 触らなかったもの

- **配分ロジック（`u_ref_shift` / clamp / `Motor_eta`）**: (2) より修正不要
- `Q_p = 80`: ⚠ 下記のとおり懸念はあるが、今回は変えない

## 検証結果

- **ビルド成功、警告ゼロ**。`text 684568 / bss 422428`
- **ホスト検査 `ftc4.cpp` 全項目 PASS（失敗 0 件）** — carry 判定 / モーメント 0 / clamp / box 天井
- ⚠ **実機未検証**

## 次に触る人へ

- ⚠ **スロットルは `u_base` 0.583〜0.633 の窓に入れること。** 上げすぎると姿勢権限が消える。
  ログの `u_base` 列で確認できる
- ⚠ **`Q_p = 80` の減衰不足（ζ = 0.25）が再び効く領域に入った。**
  285.3 g では余地 0.011 で「何を指令しても出ない」状態だったので振動は問題化しなかったが、
  271 g では権限が戻るぶん [`20260915_v2_ftc_flew_yaw_fixed_but_roll_damping_halved.md`](20260915_v2_ftc_flew_yaw_fixed_but_roll_damping_halved.md)
  の 3.1 Hz 発振が再現する可能性がある。**1 便目は `Q_p` を触らずに撮り、発振したら 250 へ**
- ⚠ **`Relax_enable = 0` のままなので relaxed hover は動かない。** 今回は純粋な静的 4 発試験
- 健全 6 発のホバー duty は 271 g で **0.516** の見込み。実測がこれから大きく外れたら、
  推力較正（スケール 0.8389）そのものを疑うこと
- 関連: [`20260916_bench_thrust_curve_overstates_flight_thrust_ftc_has_no_authority.md`](20260916_bench_thrust_curve_overstates_flight_thrust_ftc_has_no_authority.md)、
  [`20260915_uref_must_point_at_the_balanced_allocation.md`](20260915_uref_must_point_at_the_balanced_allocation.md)
