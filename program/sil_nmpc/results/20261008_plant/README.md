# 2026-10-08 プラント作り込み（6 自由度・空力・比力加速度計）の SIL 評価

本文: `docs/20261008_plant_6dof_report.md`（図は `docs/figs/20261008_plant/`）

| ファイル | 中身 |
|---|---|
| `run_ladder.sh` | プラント段階（old/legacy/sf_nodrag/sf/bf/motor/full/wind）× ideal/real × seed、ブローバック倍率の感度（4 並列） |
| `run_rp10_full.sh` | reset_period=10 の複製ファームを full プラントで（`RP10_BUILD` を指定） |
| `metrics_plant.py` | 指標（推定−真値、真の追従、水平移動、firmware ログ由来の指標）→ `plant_metrics.csv` |
| `build_figs_plant.py` | 図 1–8（図 7 は `program/NMPC_simulation/img/20261008_aero/aero_compare.npz` を読む） |
| `<plant>_<cond>_s<seed>.npz` | 各飛行。`bfx<k>_*` はブローバック係数の倍率 k、`rp10/` は再構築 10 周期 |

プラント本体は `program/sil_nmpc/host/plant6dof.py`、単体試験は `host/test_plant6dof.py`。
