# 姿勢ゼロ点の取り直しを common_duty 基準に揃える

- **日付**: 2026-08-25
- **種別**: bugfix
- **対象**: `Benzen_new/control.cpp`（`Flight_duty`）、`program/sil/host/firmware_consts.py`
- **タグ**: #Benzen_new #Flight_duty #common_duty #ゼロ点

## 背景 / 症状

M4 でモータ停止は `common_duty < Disable_duty` になったが、姿勢ゼロ点は
`T_ref / BATTERY_VOLTAGE < Flight_duty(0.05)` のままだと、スロットル比
0.045〜0.100 でモータは回るのに毎周期 `Phi_bias = Phi` となり姿勢が効かない。

## 原因 / 判断根拠

停止とゼロ点で尺度が違う。ゼロ点も PWM（`T_ref * T_ref_coe`）で見る。
値は `Disable_duty` と同じ 0.1 にする。地上ではモータ停止中にゼロ点を取り直し、
回り始めた瞬間にロックする。

## 対応

- `Flight_duty = 0.1`
- `angle_control` / `apply_auto_maneuver_refs` は既に `common_duty < Flight_duty`
- SIL の `attitude_live()` を `base_duty >= 0.1` に追従
- `control.cpp` は CRLF のまま

## 検証結果

未検証（SIL 再実行なし）。

## 次に触る人へ

モータ始動と姿勢ロックはスロットル比 ≈ 0.045 で同時。ホバー duty 0.285 より下。
