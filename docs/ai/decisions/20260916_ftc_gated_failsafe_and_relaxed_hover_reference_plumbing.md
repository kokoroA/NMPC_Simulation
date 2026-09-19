# `FS_RATE` と EKF ドリフト監視を `Ftc_mode` ゲートに、relaxed hover の参照配管を入れた（既定は無効）

- **日付**: 2026-09-16
- **種別**: design
- **対象**: `Benzen_NMPC/control.cpp`（failsafe / `ekf_drift_watch` / 参照生成 / ログヘッダ）
- **タグ**: #relaxed_hover #FTC #FS_RATE_LIMIT #EKF #ドリフト監視 #x_ref #U_ref #ランプ #Ftc_mode

## 背景

`Cr` 同定実験は翌日。それまでにできる **`Cr` 非依存の作業**として、
[`20260916_relaxed_hover_prize_is_cr_independent_and_ekf_drift_guard_will_misfire.md`](20260916_relaxed_hover_prize_is_cr_independent_and_ekf_drift_guard_will_misfire.md)
の項目 1（フェイルセーフ / EKF 監視のゲート）と項目 3（参照生成の配管）を実装した。
**要件は「健全モードに一切影響を与えないこと」。**

## 判断根拠

### (1) 緩めるのは角速度と EKF ドリフト監視だけ。姿勢フェイルセーフは残す

- `FS_RATE_LIMIT = 360 deg/s` は relaxed hover の平衡回転数（Cr 依存で 300〜775）より低い。
  立ち上がりのオーバーシュート込みのホストシミュ実測ピークは 751 deg/s → **1200 deg/s** とした
- `EKF_DRIFT_LIMIT = 5 deg/s` は `cosφ·tanθ·r` だけで −76 deg/s になる領域では 6.6% 相当 → **60 deg/s**
- ⚠ **姿勢フェイルセーフ 90° は緩めない。** relaxed hover の傾きは 4〜14° なので、
  安全網として機能したまま残る
- ⚠ 離陸ゲートは別定数 `EKF_GATE_LIMIT` を使うので、ドリフト監視を緩めても**ゲートは開きやすくならない**

### (2) 参照は「ステップ」ではなく「静的 FTC の釣り合い点からのランプ」

ホストシミュで、ステップ参照は立ち上がりで duty 飽和 10%、3 s ランプは 0%
（[`20260916_relaxed_hover_reachable_in_sim_cr_mismatch_is_the_real_blocker.md`](20260916_relaxed_hover_reachable_in_sim_cr_mismatch_is_the_real_blocker.md)）。

⚠ **ゼロからではなく「現行の静的 FTC 参照」から補間する形にした。**
`s = 0` で現行と厳密一致するので、スイッチ投入の瞬間が連続になる。

### (3) RC スイッチに割り当てなかった

`Relax_enable` は既定 0 の変数で、**ソースを書き換えて焼き直すまで作動しない**。
平衡値が `Cr` 仮定の暫定値である以上、飛行中に誤って入る経路を作らないほうがよい。

### (4) 平衡表は `Ftc_target_motor == MOTOR_FR` 専用

解いたのは FR 停止の平衡だけ。他発を止めたときに流用すると**全く別の配分**になるので、
適用側でも `Ftc_target_motor == MOTOR_FR` を条件に入れた。

## 対応

`control.cpp` のみ。すべて `Ftc_mode` または `Relax_enable` ゲート。

| 箇所 | 内容 |
|---|---|
| `FS_RATE_LIMIT_FTC` 新設 | 1200 deg/s。failsafe は `Ftc_mode ? FTC値 : 従来値` |
| `EKF_DRIFT_LIMIT_FTC` 新設 | 60 deg/s。`ekf_drift_watch()` で同様に切替 |
| `Relax_*` テーブル新設 | `enable`(0) / `x_eq` / `u_eq` / `ramp_s`(3.0) / `active` |
| 参照生成 | `relax_s` のランプ計算と、`x_ref`・`u_ref_shift` のブレンド |
| ログ | `snap.phi_ref/theta_ref/psi_ref` を **実際に追従させた `x_ref`** から取るよう変更 |
| ログヘッダ | `#fs_limits:` と `#relax:` を追加、`#ekf:` に `drift_lim_ftc_dps` を追加 |

### 却下した案

- **`Relax_enable` を RC スイッチに割り当てる**: (3) のとおり。暫定値のうちは危険
- **参照をステップで切り替える**: (2) のとおり飽和が出る
- **ログ列を増やして `relax_s` を残す**: `DATANUM` が変わりリング長が縮む。
  `psi_ref` 列が平衡値へ寄っていく様子でランプは追えるので不要と判断

## 検証結果

- **ビルド成功、警告ゼロ**。`text 684544 / bss 422428`
- **ホスト検査を新規作成**（`relaxtest.cpp`、`g++` で全 5 項目 PASS・失敗 0）:
  1. 閾値の選択が `Ftc_mode` で正しく切り替わる
  2. `relax_on` が立つのは「`Relax_enable`＝1 かつ `Ftc_mode`＝1 かつ FR 停止」の 1 通りだけ（5 ケース）
  3. **健全時は `u_hover` 0.20〜0.90 の全点で `x_ref`・`u_ref_shift` ともに不変**
  4. FTC でも **`s = 0` なら従来の静的 FTC と厳密一致**
  5. `s = 1` で `x_eq` / `u_eq` に厳密一致、**FR の絶対 duty は 0**、`U_ref` は全点 box 内
- `trim::enable = 0` を確認済みなので、ログの `phi_ref/theta_ref/psi_ref` は
  **健全時に従来と数値的に同一**（`att_trim_apply_ref` が即 return するため）
- ⚠ **実機未検証。** relaxed hover そのものは `Relax_enable = 0` なので、この状態で飛ばしても
  変わるのは「`Ftc_mode` 中の失効しきい値」だけ

## 次に触る人へ
 
- ⚠ **`Relax_x_eq` / `Relax_u_eq` は `Cr = 1.0e-4` 仮定の暫定値。同定後に必ず入れ替えること。**
  `Cr = 3.7e-4` なら Ω=400 deg/s・傾き −10°、`9.0e-4` なら Ω=300 deg/s・−14° 側へ寄る
- ⚠ **`Ftc_mode` を入れた時点で角速度フェイルセーフが 1200 deg/s に緩む。**
  静的 FTC の試験でも保護が弱くなっていることを意識すること
- ⚠ `Relax_u_eq` は**絶対 duty** なので、スロットル `u_hover` を変えても参照する動作点は動かない。
  一方 box は `[−u_hover, 0.95−u_hover]` なので、`u_hover` が極端だと `U_ref` が端に寄る
  （ホスト検査では 0.30〜0.80 で box 内を確認済み）
- 次は計画書 Step 0（`BENCH_YAW_EXCITE` で `Cr` 同定）→ Step 1（`nmpc_params.hpp` の `Cr` 更新）
- 関連: [`20260916_relaxed_hover_is_now_the_only_path_and_cr_is_step0.md`](20260916_relaxed_hover_is_now_the_only_path_and_cr_is_step0.md)、
  [`20260914_bench_yaw_excite_blocked_by_fs_rate.md`](20260914_bench_yaw_excite_blocked_by_fs_rate.md)
