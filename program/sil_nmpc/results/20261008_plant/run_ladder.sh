#!/bin/bash
# プラントの段階（ladder）× 条件。4 並列（SIL は 1 プロセス 1 インスタンスなので別プロセスで回す）
cd "$(dirname "$0")/../../host"
OUT=../results/20261008_plant
jobs=()
for pl in old legacy sf_nodrag sf bf motor full wind; do
  jobs+=("--plant $pl --cond ideal --seed 0 --out $OUT/${pl}_ideal_s0.npz")
  for s in 0 1 2 3 4 5; do jobs+=("--plant $pl --cond real --seed $s --out $OUT/${pl}_real_s$s.npz"); done
done
# ブローバックの強さ（推定値の倍率）
for k in 0 0.25 0.5 2; do
  kr=$(python3 -c "print(7.5e-3*$k)"); kp=$(python3 -c "print(2.8e-3*$k)")
  jobs+=("--plant full --cond ideal --seed 0 --set k_bf_roll=$kr k_bf_pitch=$kp --out $OUT/bfx${k}_ideal_s0.npz")
  for s in 0 1 2; do jobs+=("--plant full --cond real --seed $s --set k_bf_roll=$kr k_bf_pitch=$kp --out $OUT/bfx${k}_real_s$s.npz"); done
done
printf '%s\n' "${jobs[@]}" | xargs -P 4 -I{} sh -c 'python3 run_nmpc_eval.py {} 2>&1 | grep -v "^$"'
