# Benzen_new 自動演技の呼び出し側を 100 Hz / 400 Hz に接続

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `Benzen_new/control.cpp`, `control.hpp`
- **タグ**: #Benzen_new #PID #auto_maneuver #Flight_mode

## 背景 / 症状

`auto_maneuvers` は純粋関数のみで、`Phi_ref` / `T_ref` へ接続されていなかった。
`rate_control` が毎周期 `Flight_mode` と `T_ref` を上書きするため、角度ループだけに書くと消える。

## 原因 / 判断根拠

- 姿勢指令は 100 Hz の `angle_control`、推力は 400 Hz のスティック `T_ref` が正。
- 通常飛行は `Rref = Psi_ref` なのでヨー角速度は `Psi_ref` へ入れる。
- プロポ判定を自動中も回すと `NORMAL` に即戻る。

## 対応

- モック値 `AUTO_HORIZONTAL_TURN=20` / `AUTO_FIGURE_EIGHT=21` / `AUTO_CLIMBING_TURN=22`。
- `apply_auto_maneuver_refs()` が 100 Hz で3関数を呼び、`finished` で `NORMAL`（手動）へ戻す。
- 自動中はプロポのモード上書きを抑制。`T_ref *= (1 + thrust_delta_ratio)` をスティック代入直後に実行。
- 入場は `MOCK_AUTO_FLIGHT_MODE`（既定 0）。スロットルが低い間は `elapsed` を進めない。

## 検証結果

- ホストの `auto_maneuvers_test` は既存のまま。呼び出し側の実機飛行は未検証。

## 次に触る人へ

- 試す演技は `control.hpp` の `MOCK_AUTO_FLIGHT_MODE` を `AUTO_*` のいずれかに変える。
- プロポ割り当てに置き換えたら `MOCK_AUTO_FLIGHT_MODE` は 0 のまま、入場条件だけ足す。
- 終了直後はスティックが中立でないと手動目標がステップで入る。
