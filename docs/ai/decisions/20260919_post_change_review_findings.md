# 今回の一連の修正を通し点検した。実バグ 2 件（どちらも診断系）を直し、`Cr` が未修正のまま最大の未解決として残っていることを確認した

- **日付**: 2026-09-19
- **種別**: bugfix | investigation
- **対象**: `Benzen_NMPC/control.cpp`(logging), `nmpc_params.hpp`(Cr, mass), ビルド成果物
- **タグ**: #Benzen_NMPC #レビュー #box_n #DATANUM #Cr #ヨー減衰 #ビルド成果物

## 背景

2026-09-19 に入れた一連の変更（USB デバッグ出力 / ログ吸い出しのエッジ化 / LED カウンタ /
離陸ゲートの低スロットル条件 / `|U_lin|` ガードの出力分離としきい値 box 相対化 /
Hessian のブロック下三角 / TR1 skip キャップ 8 / ログ 3 列追加）の通し点検。

## 見つけた実バグ（修正済み）

### (1) ⚠ `box_n` が「実際には出していない解」を数えていた

`logging()` の `box_n` は `g_nmpc_ctx.ws.z(j)` を box と比較していた。
同日の修正で **安全弁 3 が `ws.z` を `U_ref` へ戻すようにしたまま**なので、
**ガードが発火した周期の `box_n` は「出していない解」の飽和数**になる。
⚠ **ガード発火周期はまさに解析したい周期**なので、放置するとログを読み違える。
→ `g_nmpc_ctx.scratch.z_applied(j)` に変更。

### (2) `g_log_cols_written` が `#ifdef USE_NMPC` の内側にあった

`USE_NMPC` 未定義ビルドでは列数が記録されずヘッダが `written=0` になる。
`#endif` の外へ移動。両構成で 33 列を書くことをスクリプトで確認済み。

## 確認して問題なかったもの

- **列数**: `USE_NMPC` の有無どちらでも `L[k++]` は **33 回** = `DATANUM = 33`。
  `#cols:` の列名も `t` + 33。`LOGROWS = 65280/33 = 1978 行`（65274 ≤ 65280）、`LogRow` は uint16_t で足りる。
- **警告**: `-Wall -Wextra -Wformat=2` で再コンパイル。**新規コードに警告なし**
  （既存の未使用変数と enum 比較のみ）。printf の書式と引数の不一致もなし。
- **行末**: `control.cpp`/`control.hpp`/`pico_copter.cpp` は CRLF、`nmpc*` は LF のまま維持。
- **`logdata_out_com()` のエッジ**: `prev` は `Arm_flag==3 && LockMode==0` でしか更新されないが、
  吸い出し中（`Arm_flag==4`）に呼ばれないので `prev` は 1 のまま保たれ、連続再発火しない。
  スティックを離せば `prev` が 0 に戻り、次の操作で再び発火する。**意図どおり。**
- **`led_control()`**: `LED_BRANCH()` は `cnt` と `led_prev_branch` を関数スコープ static として
  正しく捕まえ、末尾で `#undef` 済み。全分岐が `>=` 折り返し。
- **`tr1_skip_run`** は `uint8_t`。キャップ 8 で桁あふれなし。
- **`debug_stream_poll()` のビット詰め**: `arm(0-2) lock(3-5) gate(6) fm(7-9) ftc(10) bench(11-13)
  fs(14) cause(15-17) og(18) led(19-22) out(23-25) logout(26) lk(27) dm(28) gwhy4(29)` で重なりなし。
- **`#dbg` 行が CSV に混ざる件**: `tools/plot_onboard_log.py` は `#` 始まりを捨てるので影響なし。
  printf はすべて core0 の main から出るので行が混線することもない。
- **`LINETRACE` / `REDCIRCLE` / `unique_mission`**: 残っている参照はすべてコメント内。生きたコードなし。

## ⚠ 未修正のまま残っている最大の問題: `Cr = 0.005`

減衰は **2 乗則** `−Cr·r·|r|`。`r = 200 deg/s` での自己減衰は

| `Cr` | 減衰トルク | `ṙ` |
|---|---|---|
| **0.005（ファーム値）** | 0.0609 N·m | **−1164 deg/s²** |
| 3.7e-4（実測上限） | 0.0045 N·m | −86 deg/s² |
| 9.5e-5（実測下限） | 0.0012 N·m | −22 deg/s² |

⚠ **モデルは実測の 14〜53 倍の「ヨーは勝手に止まる」を信じている。**
N=14 × 10 ms = 140 ms の予測ホライズン全体でこの過大減衰が効くので、
**NMPC はヨー偏差に対して構造的に過小な操作しか出さない。**

`20260919_payload_drop_crash_ulin_guard_zeroes_du.md` で「ヨーを捨ててピッチを取った」原因を
`Q_r = 40` vs `Q_theta = 1000` の重み比に求めたが、⚠ **重みだけでなくモデルも効いている。**
`Cr` が正しければ「放っておくと止まらない」と予測するので、同じ重みでももっと早く手当てする。

これは 2026-09-16 に **「`Cr` 同定が Step 0」** と書かれて以来 3 日間未着手のままである
（`20260916_relaxed_hover_is_now_the_only_path_and_cr_is_step0.md`）。

## ⚠ そのほか気づいた点（今回は直していない）

- **`mass = 0.285f`** だが機体は 271 g（貨物込みで 296 g）。⚠ **制御には入らない**ので実害はないが、
  ログヘッダに `mass=0.285` と出るので**後日の解析者が実重量と誤認する**。
  （`20260917_payload_drop_mass_param_is_inert.md`）
- **`T_max = 0.81`** に対し飛行較正の最大推力は `0.7052 N`。`B_c` は `T_max` でスケールするので
  **モデルは制御権限を約 15% 過大に見ている。**
- **`C_QT` のコメントが古い**（「≈ 0.0230 (CQ=1.2e-8)」とあるが実際は `1.54e-8/5.21e-7 = 0.0296`）。
- **`Cp = Cq = 0.025` は一度も実測していない**（`20260905_cr_upper_bound_and_yaw_hang_test.md`）。
- ⚠ **`ekf_drift_update()` は 2 か所で早期 return する**（reseed 直後、`|θ| > 60°`）。
  その周期は離陸ゲートの判定自体が走らないので、`gwhy` が前周期の値のまま残る。

## ⚠ ビルド成果物の取り違え（運用上の地雷）

`build/` と `build_fresh/` の 2 つがあり、**今日すでに 1 度これで事故っている**
（`0919_v1〜v3` は計装の無い古いファームで取られた疑いが濃い）。
点検中も `build_fresh` が 3 分古い状態だったので**両方を再ビルドして同一にした**。

```
build/pico_copter.elf        text 687712  bss 422776
build_fresh/pico_copter.elf  text 687712  bss 422776   ← 一致
```

## 検証結果

- 列数・ビット詰め・警告・行末はスクリプトとコンパイラで確認。
- ⚠ **実機未検証。** (1) の修正は診断値だけなので飛行挙動には影響しない。

## 次に触る人へ

- ⚠ **起動直後に `#dbg_cols:` が出るか、吸い出しヘッダが `cols=33 written=33` かを毎回確認すること。**
  前者が無ければ古いファーム、後者がずれていればログが壊れている。
- ⚠ **`Cr` を直すのが次の一手。** 重みをいじる前にモデルを直すこと。
- 関連: [`20260919_payload_drop_crash_ulin_guard_zeroes_du.md`](20260919_payload_drop_crash_ulin_guard_zeroes_du.md)、
  [`20260916_relaxed_hover_is_now_the_only_path_and_cr_is_step0.md`](20260916_relaxed_hover_is_now_the_only_path_and_cr_is_step0.md)
