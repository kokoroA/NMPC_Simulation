#!/bin/bash
# SIL 評価の一括実行（1 プロセス 1 インスタンスなので条件ごとに別プロセス）
cd "$(dirname "$0")/../../host"
OUT=../results/20261008
python3 run_nmpc_eval.py --cond ideal --seed 0 --out $OUT/ideal_s0.npz
for c in delay noise real; do
  for s in 0 1 2; do
    python3 run_nmpc_eval.py --cond $c --seed $s --out $OUT/${c}_s${s}.npz
  done
done
