# Benzen_Perfect2025 への自動旋回移植は可能（丸ごとコピーは不可）

- **日付**: 2026-08-26
- **種別**: investigation
- **対象**: `Benzen_pico1`（自動旋回）→ `Benzen_Perfect2025`
- **タグ**: #Benzen_Perfect2025 #Benzen_pico1 #auto_maneuver #unique_mission #移植

## 背景 / 症状

`Benzen_pico1` の自動旋回（水平旋回・8の字・上昇旋回）を `Benzen_Perfect2025` に
「丸ごと」入れたい。可能か。

## 原因 / 判断根拠

**結論: 可能。ただし `control.cpp` を丸ごと置き換えてはならない。**

生成器 `auto_maneuvers.{hpp,cpp}` は副作用のない純粋関数で、両系統の
`Phi_ref` / `Theta_ref` / `Psi_ref(=Rref)` / `T_ref` に載る。
呼び出し側だけ Perfect2025 の推力尺度・既存ミッションと衝突しないよう配線する。

過去に却下済み: `unique_mission()`（宙返り）を自動旋回の器にする案
（`20260824_benzen_new_auto_maneuvers.md`）。Perfect2025 ではこの関数が
**現役のフリップ**なので、消さず共存させる。

### そのままコピーできるもの

- `auto_maneuvers.hpp` / `auto_maneuvers.cpp`（ホストテスト含む）
- `AUTO_*` / `MOCK_AUTO_FLIGHT_MODE` マクロ
- LED 色分け（`rgbled_blue/green/pink` は Perfect2025 にもある）

### 移植時に合わせる必要があるもの

1. **推力尺度が違う。** pico1 は `T_ref = 0.5*BATTERY_VOLTAGE*stick` を
   `T_ref*T_ref_coe` で PWM 化。Perfect2025 は `T_ref` が 0〜1 の duty そのもの
   （ミキサに `T_ref_coe` を掛けない）。
   `apply_auto_maneuver_refs` のスロットルゲートは pico1 の `common_duty` を
   コピーせず、Perfect2025 既存の
   `T_ref < Flight_duty * BATTERY_VOLTAGE_BASE/BATTERY_VOLTAGE_REAl` に合わせる。
   上昇旋回の `T_ref *= (1 + thrust_delta_ratio)` は尺度非依存なのでそのまま使える。
2. **トリム。** Perfect2025 は毎周期 `Phi_trim≈-0.6deg` / `Theta_trim=4deg` を足す。
   自動指令を上書きするなら trim を残すか絶対指令にするかを決める必要がある
   （pico1 は trim=0 なので問題にならなかった）。
3. **`unique_mission`（Chdata[FLIP]）と排他。** 自動中に FLIP が立つと
   400 Hz で角速度指令を奪う。自動モード中は `unique_mission` を呼ばない。
4. **プロポによる `Flight_mode=NORMAL` 上書き抑制**（pico1 と同じ）。
   入場は当面 `MOCK_AUTO_FLIGHT_MODE`。Perfect2025 の ch4 は FLIP 占有、
   ch5/7/8 は投下（`high_fall`）が使う。プロポ割り当ては空きチャンネルが無い。

### 丸ごとコピーしてはいけないもの

- `control.cpp` 全体（フリップ・投下・PID・ミキサ・トリムが消える）
- pico1 の PID ゲイン（Perfect2025 のロールは kp=2.1・微分あり。pico1 は kp=1・積分なし）

## 検証結果

コード差分の読み取りのみ。ビルド・実機は未実施。

## 次に触る人へ

実装するなら: 生成器コピー → CMake 追加 → `apply_auto_maneuver_refs` を
Perfect2025 の T_ref 尺度で接続 → 自動中はスティック上書きと FLIP フリップを抑制。
実機軌道は pico1 側も未検証。開ループなので風・初速でずれる前提。
