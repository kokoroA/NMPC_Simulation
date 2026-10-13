# 2026-10-13 225 g の耐故障制御（1 発停止）SIL 評価

本文: `docs/20261013_ftc_225g_sil_eval.md`（図 `docs/figs/20261013_ftc225/`、図スクリプト `docs/analysis/20261013_ftc225/figs_ftc225.py`）

| ファイル | 中身 |
|---|---|
| `jobs_*.txt` | `host/run_jobs.py` に渡すジョブ（行頭の `SIL_SCRIPT=run_ftc_eval.py`、`SIL_BUILD_DIR` は作成者の環境のパス） |
| `runs/` | 1 本 1 ファイル（254 本、311 MB）。⚠ コミットしない。名前は `<ファーム>_<止めた発>_<air/gnd>_m<質量>[_条件]_s<種>.npz` |
| `analyze_ftc.py` | 集計（`ftc_summary.csv`, `summary_table.txt`）と指令の向きの関数 `cmd_stats` |
| `patch_ftcqr.py` | ファームの**複製**に「FTC 中だけ Q_r を差し替える」を入れる（`-DSIL_FTC_QR=2.0f` でビルド）。実機ファームには当てていない |

ビルド: `base` = 現行ファーム無改造、`qr10/qr2/qr0` = 複製に patch_ftcqr.py ＋ `-DSIL_FTC_QR=…`。
条件: `--set` なしの full プラント、`--cond shake`。`_I08` = プラント慣性 0.8:0.8:0.75、`_dd02` = 検知 0.2 s 遅れ、
`_wind` = 1.5 m/s＋突風 0.5、`_yaw` = プラントの Cr×3.7・C_QT×0.69、`_ideal` = 雑音・遅れなし。
