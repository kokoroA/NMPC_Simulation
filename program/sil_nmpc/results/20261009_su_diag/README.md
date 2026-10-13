# 2026-10-09 TR1/SR2 が Su・H をどれだけ壊しているかの直接測定（SIL）

本文: `docs/20261009_su_exact_explained.md` §4（図 `docs/figs/20261009_su_exact/`、スクリプト `docs/analysis/20261009_su_exact/figs_su_exact.py`）

- `su_diag_hook.patch`: Benzen_NMPC の `nmpc.cpp`（2026-10-09 時点）に当てる診断。**ファームの複製にだけ当てること。**
  毎周期、ソルバが使う Su・Sx・H と、同じ (x0, U_lin) で厳密に作ったものを比べて `SIL_DIAG_FILE` に 1 行書く。ソルバの状態は変えない。
- `d0_s{0,1,2}.csv`: `-DNMPC_SU_EXACT=0`（従来 TR1/SR2）、`d1_s*.csv`: `=1`（Su 毎周期厳密）。
  条件はすべて `run_nmpc_eval.py --plant full --cond shake --seed {0,1,2}`。3 本とも墜落なし。
- 列の意味は本文の付録を参照。
