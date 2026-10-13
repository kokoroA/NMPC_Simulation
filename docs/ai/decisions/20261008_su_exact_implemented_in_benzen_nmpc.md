# Benzen_NMPC に「Su/Sx を毎周期厳密に作り直す」を実装（既定 ON、`NMPC_SU_EXACT=0` で従来の TR1/SR2）

- **日付**: 2026-10-08
- **種別**: design
- **対象**: `Benzen_NMPC/nmpc_params.hpp`（`rti::su_exact_each_cycle`, マクロ `NMPC_SU_EXACT`）、`nmpc.cpp`（`nmpc_step` の通常経路）、`docs/NMPC_REPORT.md`（§1.3, §2.3, §5.4）
- **タグ**: #Benzen_NMPC #TR1 #SR2 #Su #ソルバ #逆向き指令 #NMPC_SU_EXACT #計算時間 #SIL #予稿

## 背景

ユーザ依頼「Benzen_NMPC の方も Su を毎周期作り直す変更を実装してください」。
根拠は SIL の比較（`20261008_solver_bursts_come_from_tr1_sr2_updates_su_exact_fixes.md`）。

## 対応

- `nmpc_step` の通常経路を `rti::su_exact_each_cycle` で分岐。true のとき `full_rebuild_sx_su(ctx, x0, U_lin)` だけを呼び、TR1/SR2 は通らない。
  H（LLT）は触らない（`reset_period=100` ごとのフル再構築のときだけ作る）。
- ログ上は通常経路（solver_code の path=0）として記録されるよう `last_tr1_applied=1` にし、割線の診断値（tr1_c, secant_nsq, tr1_skip_run）は 0 にした。
  `cnt_tr1_applied` / `cnt_tr1_skipped` は増やさない（TR1 が動いた回数という意味を保つ）。
- スイッチはマクロ `NMPC_SU_EXACT`（既定 1）→ `constexpr bool`。0 で従来と完全に同じ。
- Benzen_NMPC はどの git にも追跡されていない（pico-sdk の中で未追跡）ので、変更前のファイルを
  `program/sil_nmpc/results/20261008_fix/firmware_backup_before_su_exact/` に退避した。
- 採らなかった案: 再構築 10 周期（効くが停止が 10 倍）、差分更新なし（同等に良いが線形化点が最大 1 s 古い）。

## 検証結果

- 実機向けビルド（`build_fresh`）成功、警告なし。text 687,704 → 685,512、bss 422,776（不変）。`pico_copter.uf2` を更新。
- SIL（本体を直接ビルド）で、**既定（ON）は比較用スイッチの「Su 毎周期厳密」とビット単位で一致**（shake 3 本・乱流 seed4・風 seed0 の 5 本、ログの診断列以外とプラント真値の差 0.0）。
  **`NMPC_SU_EXACT=0` は従来とビット単位で一致**（乱流 seed4 は同じ t=12.51 で墜落）。
- ON のログの path 内訳: 通常 1737 / 再構築 18（約 1 %）。
- **実機では未検証。計算時間（iter_us）は未測定**（積和の見積もりで 約 74k → 約 58k/周期）。

## 次に触る人へ

- 最初の実機確認はベンチモードで `iter_us` と `loop_us`。次に地上・低空ホバーで逆向き指令・|Δduty| を従来版と比べる（`NMPC_SU_EXACT=0` のビルドを並べて持っておく）。
- 予稿 `paper.typ` は手法を「RTI + TR1 + SR2」と書いている。実機の既定が変わったので、結果を出すなら手法の記述と計算時間の表を更新すること。
- TR1/SR2 のコードは残してある（`NMPC_SU_EXACT=0`）。消すのは実機で確認が取れてから。
- 関連: `20261008_solver_bursts_come_from_tr1_sr2_updates_su_exact_fixes.md`、`docs/20261008_ekf_and_solver_fix_study.md`。
