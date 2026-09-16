# ヨー加振が始まらない直接原因はスロットルではなく、手回しが FS_RATE をラッチしたこと

- **日付**: 2026-09-14
- **種別**: investigation
- **対象**: `Benzen_NMPC/control.cpp`（`bench_excite_update` / `failsafe_check` / `led_control`）、ログ `tools/0914_yaw_V1.csv`
- **タグ**: #Benzen_NMPC #bench #FS_RATE #ヨー吊り #デッドマン #LED #sysid

## 背景 / 症状

`Bench_mode = BENCH_YAW_EXCITE` を焼いた状態でアームし、スロットルを動かしても加振シーケンスが始まらない（モータが回らない）。

## 原因 / 判断根拠（ログで確認済み）

`0914_yaw_V1.csv`（Wi-Fi 10 Hz、約 115 s、全行 `arm=2`）:

| 観測 | 意味 |
|---|---|
| 全行 `nmpc=0`、全 duty = **0.0**（`Disable_duty=0.1` ではない） | 加振モードの待機経路に入っている。`Bench_mode` の書き換えと書き込みは成功 |
| 全行 `T_ref = 0` | この記録中、スロットル CH3 は `CH3MIN` のまま。デッドマンは一度も開いていない |
| `t≈11.9 s` で `failsafe` が **0 → 4（FS_RATE）** | 角速度フェールセーフがラッチ。以後 PWM は `motor_stop()` が毎周期勝つ |
| 同刻の `r = −408 deg/s`（上限 360） | 誤検知ではない。直前 2 秒は既に `r ≈ −250 deg/s` |

`t=8 s` まで `r ≈ 0`、`t=9 s` で突然 −131 deg/s。モータ duty は 0 のままなので、**スイベル上で機体を手回しした自由減衰**である。減衰曲線はその後 80 秒かけて −400 → −40 deg/s まで落ちており、軸受チェックそのものに見える。

加振モードでも **角速度の脱出条件は意図的に残してある**
（[`20260906_bench_excitation_mode.md`](20260906_bench_excitation_mode.md)、
[`20260914_yaw_hang_test_poweron_orientation_and_failsafe.md`](20260914_yaw_hang_test_poweron_orientation_and_failsafe.md)）。
吊り試験のヨーは実験量そのもので、手回しも加振も 360 deg/s を簡単に超える。**残した安全弁が、この試験の正常動作を止めている。**

ラッチ後はディスアームするまで解除されない。LED は `Arm_flag==2 && Bench_mode != BENCH_OFF` が最優先で **ピンクのまま**なので、フェールセーフ成立が一目では分からない。

スロットル側のゲート `T_ref * T_ref_coe >= Flight_duty(0.05)` は、このログでは一度も成立していない。後からスティックを上げても、ラッチ中は 400 Hz 側が PWM を 0 に戻す。ソフト上の `Bench_phase` は進み得るが、モータは回らない。

## 対応

コードはまだ変えていない（分析依頼）。現場でシーケンスを通す最短手順:

1. ディスアーム → 再アーム（`Failsafe_flag` クリア）。ピンク点滅を確認
2. **アーム後は機体を手で回さない**（軸受確認はアーム前、または `BENCH_MOTORS_OFF` で別ラン）
3. グラフの `T_ref` が 0 から動くこと、`failsafe` が 0 のままであることを見てからスロットルを上げる

加振モードで `FS_RATE` を残すかは設計判断が要る。ヨー吊りでは `r` が目的変数なので、少なくともヨー軸は外す／上限を実験上限（要領書の中断 250 deg/s）より十分上へ上げる必要がある。ロール・ピッチと過大 G は治具破損検知として残せる。**未実装。**

## 検証結果

ソースと CSV の突合のみ。実機での再試行は未実施。

確認した箇所:

- `control.cpp:2172-2188`（デッドマン）、`1942-1944`（`Failsafe_flag` で `motor_stop`）、`1387-1393`（加振中は姿勢のみ除外、レートは残る）
- `control.cpp:638-646`（加振中 LED はフェールセーフを隠す）
- `tools/0914_yaw_V1.csv`（`failsafe=4` が 929 行中 810 行、`T_ref` は全行 0）

## 次に触る人へ

- ⚠ **ピンク点滅は「加振モードで武装している」だけを示す。フェールセーフ成立とは独立。**
- 要領書の軸受チェック（手回し自由減衰）をアーム後にやると、必ずこの罠に落ちる。
- `T_ref` がグラフで 0 のままなら、動かしているのはスロットルではない。CH3 を疑う。
- 関連: [`20260906_bench_excitation_mode.md`](20260906_bench_excitation_mode.md)、
  [`20260914_yaw_hang_test_poweron_orientation_and_failsafe.md`](20260914_yaw_hang_test_poweron_orientation_and_failsafe.md)
