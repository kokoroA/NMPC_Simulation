# 加振モードでは角速度フェールセーフも外す。ヨー吊りでは r が実験量そのもの

- **日付**: 2026-09-14
- **種別**: design | bugfix
- **対象**: `Benzen_NMPC/control.cpp` `failsafe_check()`、`docs/20260905_yaw_damping_experiment_protocol.md` §9
- **タグ**: #Benzen_NMPC #bench #FS_RATE #ヨー吊り #failsafe

## 背景 / 症状

[`20260914_bench_yaw_excite_blocked_by_fs_rate.md`](20260914_bench_yaw_excite_blocked_by_fs_rate.md) のとおり、
`BENCH_YAW_EXCITE` 武装後にスイベル上で手回しすると `r = −408 deg/s` で `FS_RATE` がラッチし、
スロットルを上げても PWM は `motor_stop()` された。ユーザ指示は「加振中は角度だけでなく角速度のリミッターも外す」。

## 原因 / 判断根拠

**2016-09-06 の「姿勢だけ外す」は、この試験の入力と矛盾していた。**
姿勢を外した根拠は「治具拘束中は飛び去らない」。同じ根拠が角速度にも当てはまり、
しかもヨー吊りでは `r` が測りたい量で、手回しも加振も 360 deg/s を容易に超える
（要領書の中断目安は 250 deg/s、ファーム上限は 360）。
残した安全弁が試験の正常動作を止めていた。

**実装は `BENCH_YAW_EXCITE` 専用分岐にはしない。** 姿勢側と同じ `Bench_mode == BENCH_OFF` のときだけ
姿勢・角速度を評価する。`BENCH_MOTORS_OFF` は手回し自由減衰そのもので、同じラッチを踏む
（0914_yaw_V1 がまさにそれ）。過大 G と受信途絶は治具破損・リンク断の検知として残す。

通常飛行（既定 `BENCH_OFF`）の経路は 1 文字も変わらない。

## 対応

`failsafe_check()` の姿勢/角速度判定を `if (Bench_mode == BENCH_OFF) { ... }` で括った。
要領書 §9 の表を「角速度も無効 / 過大G・受信は残す」に更新し、旧表記 60°/300 deg/s を 90°/360 deg/s に直した。

## 検証結果

**未検証（実機未再試行）。** ソース上は加振中に `FS_ATTITUDE` と `FS_RATE` へ到達する経路が消えたことだけ確認。
受信途絶と過大 G は加振中も従来どおり。ビルド・書き込み後、ピンク点滅のままスロットルを上げて
duty が `Bench_u0` になることを見る。既にラッチしている機体は一度ディスアームが必要。

## 次に触る人へ

- ⚠ **加振モードのまま自由飛行してはならない。** 姿勢も角速度も止まらない。治具固定が前提。
- LED はフェールセーフ成立を隠したまま（ピンク優先）。今回は触っていない。
- 関連: [`20260906_bench_excitation_mode.md`](20260906_bench_excitation_mode.md)、
  [`20260914_bench_yaw_excite_blocked_by_fs_rate.md`](20260914_bench_yaw_excite_blocked_by_fs_rate.md)
