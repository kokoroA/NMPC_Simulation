#!/bin/bash
# 揺れの較正（full プラント・real 条件の遅延/据え置きはそのまま、雑音と外乱だけを振る）
cd "$(dirname "$0")/../../host"; OUT=../results/20261008_shake/calib; mkdir -p $OUT
jobs=()
add() { for s in 0 1 2; do jobs+=("--plant full --cond real --seed $s $2 --out $OUT/$1_s$s.npz"); done; }
add A "--gyro 0.02 --acc 75 --set proc_sigma=0.35:0.22:0.30"
add B "--gyro 0.02 --acc 75 --set proc_sigma=0.28:0.17:0.22"
add C "--gyro 0.02 --acc 75 --set proc_sigma=0.22:0.13:0.18 gust_sigma=0.4"
add D "--gyro 0.025 --acc 75 --set proc_sigma=0.30:0.18:0.25 gust_sigma=0.25"
printf '%s\n' "${jobs[@]}" | xargs -P 4 -I{} sh -c 'python3 run_nmpc_eval.py {} 2>&1 | grep -v "^$"'
