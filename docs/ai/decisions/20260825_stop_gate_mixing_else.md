# 停止判定のあとでのみミキシングする構造へ

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `Benzen_new/control.cpp` `rate_control()`
- **タグ**: #Benzen_new #Disable_duty #common_duty #mixing

## 背景 / 症状

停止判定は既に `common_duty = T_ref * T_ref_coe` 対 `Disable_duty` になっていたが、
ミキシングは判定の**前**で毎周期実行されていた。ユーザー指示は
「閾値未満なら stop、else で既存ミキシング」の構造。

## 原因 / 判断根拠

停止中に duty を組み立てても `motor_stop()` で捨てる。構造を指示どおりに揃える。
比較式自体は M4 で直済み（電圧比との混同を解消）。

## 対応

- `if (common_duty < Disable_duty)`: `motor_stop()` と PID リセットのみ。
- `else`: ミキシング、0.1〜0.95 クランプ、`set_duty_*`。
- `control.cpp` は CRLF のまま（バイナリ置換）。
- SIL の `firmware_consts.motors_running()` を `common_duty` 基準に合わせて写した。

## 検証結果

未検証（実機・SIL 再実行なし）。条件式の数値は M4 と同じ。

## 次に触る人へ

`Flight_duty` はまだ `T_ref / BATTERY_VOLTAGE`。スロットル比 0.045〜0.100 では
モータは回るが姿勢ループがゼロ点を取り続ける。M4 の発見 A が残っている。
