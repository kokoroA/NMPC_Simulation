# 2026-10-08 SIL 評価（現行 Benzen_NMPC の通常飛行）

本文: `docs/20261008_sil_nmpc_flight_eval.md`（図は `docs/figs/20261008_sil/`）

| ファイル | 中身 |
|---|---|
| `run_all.sh` | ideal / delay / noise / real の一括実行（実機ファーム） |
| `run_ablation.sh` | 実機ファーム vs reset_period=1 の比較（`RP1_BUILD` に複製ファームのビルド dir） |
| `metrics.py` | 1 本ごとの指標 → `metrics.csv`（実機 0919_v5/v6/v8 も同じ定義で） |
| `wrongsign.py` | 逆向き指令の割合（s = 4(角度−参照)+角速度 と指令モーメントの符号） |
| `compare.py` | ビルド（rp1/10/30/100）× 条件の集計 → `ablation_summary.csv`、再構築からの周期数別 |
| `build_figs.py` | 図 1–7 |
| `*_s<seed>.npz` | 実機ファーム（rp100）の各飛行。`ablation_rp{1,10,30}/` は reset_period を変えた複製ファーム |
| `bug_legacy_order_ideal.npz` | 修正前のモータ順を再現した飛行（図 1 左） |

reset_period を変えた複製は `Benzen_NMPC` を rsync で複製し `nmpc_params.hpp` の `reset_period = 100;` を 1/10/30 に置換して
`cmake -S program/sil_nmpc -B <dir> -DFW_DIR=<複製> -DCMAKE_BUILD_TYPE=Release` でビルドした（実機ファームは無変更）。
