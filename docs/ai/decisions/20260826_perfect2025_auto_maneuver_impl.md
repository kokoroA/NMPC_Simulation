# Perfect2025 に自動旋回を移植し、ch4/5/7 で入場する

- **日付**: 2026-08-26
- **種別**: design
- **対象**: `Benzen_Perfect2025/`（`auto_maneuvers.*`, `control.cpp`, `radio.hpp`）
- **タグ**: #Benzen_Perfect2025 #auto_maneuver #プロポ #unique_mission

## 背景 / 症状

`Benzen_pico1` の自動旋回を Perfect2025 に載せる。T_ref は 0〜1 duty のまま、
ゲインとトリムは触らない。スイッチは ch4=水平旋回、ch5=上昇旋回、ch7=8の字。

## 原因 / 判断根拠

調査 `20260826_perfect2025_auto_maneuver_port.md` の結論どおり、生成器はコピー、
呼び出し側だけ Perfect2025 の推力尺度に合わせる。

ch4 はもともと `unique_mission`（宙返り）、ch5/7 は `high_fall` 投下に使っていた。
ユーザー指定のスイッチ割り当てを優先し、自動中はフリップ呼び出しを外し、
投下判定も ch4/5/7 が立っている間は走らせない。`unique_mission` 本体は残した
（ラジオからは呼ばれない）。

トリムは手動と同じ `-0.6deg / +4deg` を自動指令に加算する。ゲインの
`set_parameter` は変更していない。

入場は立ち上がり、終了またはスイッチ OFF で `NORMAL`。スイッチを押しっぱなしで
終わったあとに再入場しない（consumed）。同時押しは 4→5→7 の優先。

## 対応

- `auto_maneuvers.{hpp,cpp}` をコピー。CMake に追加。
- `T_ref` はスティック 0〜1 のまま。`T_ref *= (1 + thrust_delta_ratio)` のみ。
- スロットルゲートは `T_ref < Flight_duty * BATTERY_VOLTAGE_BASE/BATTERY_VOLTAGE_REAl`。

## 検証結果

- ホスト `auto_maneuvers_test`: all passed。
- 実機ビルド・飛行は未検証（このマシンの CMake cache は別環境の `PICO_SDK_PATH`）。

## 次に触る人へ

- 閾値は既存どおり `Chdata > 200`。
- 終了直後はスティックが中立でないと手動目標がステップで入る。
- 宙返りを ch4 に戻すなら `unique_mission` 呼び出しを復活させ、水平旋回を別チャンネルへ。
