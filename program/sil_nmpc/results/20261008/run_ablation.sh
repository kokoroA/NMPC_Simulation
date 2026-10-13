#!/bin/bash
# 標準ビルド（TR1/SR2 低ランク更新 + 100 周期ごと再構築）と、毎周期フル再構築ビルド（reset_period=1）の比較。
# 毎周期ビルドは SIL_BUILD_DIR で指定する（ファーム複製の nmpc_params.hpp だけを 1 行変えたもの）。
cd "$(dirname "$0")/../../host"
OUT=../results/20261008
RP1_BUILD=${RP1_BUILD:?set RP1_BUILD to the reset_period=1 build dir}
for c in noise real_nh; do
  for s in 0 1 2 3 4 5; do
    [ -f $OUT/${c}_s${s}.npz ] || python3 run_nmpc_eval.py --cond $c --seed $s --out $OUT/${c}_s${s}.npz
    SIL_BUILD_DIR=$RP1_BUILD python3 run_nmpc_eval.py --cond $c --seed $s --out $OUT/ablation_rp1/${c}_s${s}.npz
  done
done
