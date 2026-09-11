# Benzen_new 自動旋回・8の字・上昇旋回の目標値生成

- **日付**: 2026-08-24
- **種別**: design
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_new/auto_maneuvers.{hpp,cpp}`
- **タグ**: #Benzen_new #PID #auto_maneuver #開ループ

## 背景 / 症状

`Benzen_new` は位置制御を持たず、姿勢 PID のみで飛行する。自動水平旋回・8の字・上昇旋回を
モード遷移から切り離して実装する必要があった。既存 `unique_mission()`（宙返り案）は
カウンタ未初期化・時間単位混在・生成値が制御参照へ未接続であり、流用しない判断。

## 原因 / 判断根拠

- 位置・速度フィードバックがないため、幾何学的な軌道追従は保証できない。
- 現行 `angle_control()` は `Phi_ref`/`Theta_ref` を角度 PID へ、`Psi_ref` を実質ヨー角速度
  (`Rref`) として使う（`Flight_mode != LINETRACE` 時）。
- 上昇は最終的に高度 PID へ移行予定だが、現段階では `T_ref = T_baseline * (1 + thrust_delta_ratio)`
  の開ループ推力補正で対応する。

## 対応

- 副作用のない純粋関数モジュール `auto_maneuvers` を新設。
  - `make_horizontal_turn`: 前傾 + 同符号バンク + ヨー角速度を smoothstep ランプで生成。
  - `make_figure_eight`: 前傾維持、`sin(2πt/T)` でバンク・ヨー角速度を正負反転（交差点で 0）。
  - `make_climbing_turn`: 水平旋回に推力比率補正を重畳。
- 呼出側は 100 Hz で `elapsed_s` を渡し、返値を `Phi_ref`/`Theta_ref`/`Rref`/`T_ref` へ適用する。
- `control.cpp` へのモード組込みは行わない（ユーザー側で実装）。

## 検証結果

- ホスト境界テスト `auto_maneuvers_test.cpp`: 全項目 pass。
- `PICO_SDK_PATH` 指定で `Benzen_new/build_fresh` から `pico_copter` ビルド成功。
- 実機飛行・軌道形状は未検証。風・初速・推力補正量で軌道がずれる前提。

## 次に触る人へ

- モード遷移側で `elapsed_s` の起点リセットを必ず行うこと。
- 上昇旋回の `climb_thrust_ratio` は小さく始め、拘束試験で調整すること。
- 高度 PID 導入後は `thrust_delta_ratio` を高度ループ出力へ置き換える想定。
- デフォルト旋回角 360°・8の字周期 8 s×2 は保守値。実機に合わせて config を調整すること。
