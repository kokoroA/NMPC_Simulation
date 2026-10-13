#!/bin/bash
# 前回の結論（再構築 10 周期 ≫ 100 周期）が、作り込んだプラント（full）でも成り立つか。
# RP10_BUILD: reset_period=10 にしたファーム複製のビルド dir（../20261008/README.md 参照）
cd "$(dirname "$0")/../../host"
OUT=../results/20261008_plant/rp10
mkdir -p $OUT
RP10_BUILD=${RP10_BUILD:?set RP10_BUILD}
for s in 0 1 2 3 4 5; do echo "--plant full --cond real --seed $s --out $OUT/full_real_s$s.npz"; done | \
  SIL_BUILD_DIR=$RP10_BUILD xargs -P 4 -I{} sh -c 'python3 run_nmpc_eval.py {} 2>&1 | grep -v "^$"'
