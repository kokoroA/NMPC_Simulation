# 2026-10-08 EKF のずれ・ソルバの暴れの対策比較（SIL）

本文: `docs/20261008_ekf_and_solver_fix_study.md`（図 `docs/figs/20261008_fix/`）

## ファーム複製の作り方（実機ファームは変更しない）
1. `Benzen_NMPC` を rsync で複製（build*, tools, docs, .git を除く）。
2. `sil_variant_switches.patch` を当てる（⚠ `control.cpp` は CRLF なので `patch --ignore-whitespace` か改行を LF に揃えてから）（`nmpc.cpp` の差分更新部分と `control.cpp` の `kalman_filter()` に比較用スイッチ。既定値で現行と同一。ビット一致を確認済み）。
3. `cmake -S program/sil_nmpc -B <dir> -DFW_DIR=<複製> -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_FLAGS="<フラグ>"`

| ビルド | フラグ |
|---|---|
| base | なし |
| suexact（Su 毎周期厳密・H は周期再構築） | `-DSIL_SOLVER_VARIANT=1` |
| noupd（差分更新なし） | `-DSIL_SOLVER_VARIANT=2` |
| rx10 / rx100（加速度計 R ×10 / ×100） | `-DSIL_EKF_RSCALE=10.0f` / `100.0f` |
| gate（\|‖a‖−g\|>1 m/s² で R ×100） | `-DSIL_EKF_GATE=1.0f` |
| suexact_rx10 | `-DSIL_SOLVER_VARIANT=1 -DSIL_EKF_RSCALE=10.0f` |
| rp10（再構築 10 周期） | 別の複製で `nmpc_params.hpp` の `reset_period` を 10 に（`../20261008/README.md`） |

## ファイル
| ファイル | 中身 |
|---|---|
| `jobs.txt`, `jobs_ekf.txt`, `jobs_combo.txt` | `host/run_jobs.py` に渡すジョブ（SIL_BUILD_DIR は作成者の環境のパス） |
| `analyze_fix.py` | 集計（`fix_summary.csv`）と図 12–14 |
| `runs/` | 各飛行。`ekf_bias003/` はバイアス漂流 0.003 で離陸ゲートが 3/12 本拒否した回（参考） |
| 現行ソルバの shake/gust/wind | `../20261008_shake/runs/` を参照 |
