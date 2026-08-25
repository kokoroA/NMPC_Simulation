# SIL M3: ログと描画、および T_ref 係数の訂正

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `program/sil/host/{logger.py,plots.py,firmware_consts.py,run_m3_demo.py,test_m3_logging.py}`
- **タグ**: #SIL #M3 #ログ #描画 #npz #T_ref #スロットル #T_max #フォント

## 背景 / 症状

M2 で閉ループは成立したが、**挙動が数値でしか見えず**、
異常の発見も傾向の把握もできなかった。全状態を落とさず記録し、
後から描画できる基盤が要る。

あわせて、M2 までホスト側が `T_ref = 0.7 * BATTERY_VOLTAGE * ratio` を前提にしていたが、
**実機の実効値は 0.5 係数**であるとの指摘を受けた。

## 原因 / 判断根拠

### T_ref の係数は 0.5 が正しい

`control.cpp` には 2 か所ある。

| 行 | 式 | 実効 |
|---|---|---|
| 833 | `T_ref = 0.7 * BATTERY_VOLTAGE * ratio` | 高度制御ブロック内。後段で上書きされる |
| **995** | `T_ref = 0.5 * BATTERY_VOLTAGE * ratio` → `T_ref *= (1 + g_thrust_delta_ratio)` | **`rate_control()` 内・400 Hz。これが実効** |

line 995 は自動機動の上昇推力補正 `g_thrust_delta_ratio` も掛けており、
こちらが最終値になる。ホスト側の定数を `firmware_consts.py` に集約し、
**control.cpp の写しであることを明記**したうえで 0.5 に統一した。

### 使えるスロットル帯が狭いことが判明した

`duty = T_REF_THROTTLE_GAIN × BATTERY_VOLTAGE × ratio × T_ref_coe` より:

| | 値 |
|---|---|
| モータが回り始めるスロットル比 | **0.200** |
| duty が上限 0.95 に張り付くスロットル比 | **0.428** |
| モータ始動時の duty（下限） | **0.444** |

**使えるのはストロークの 23% だけ**で、始動と同時に duty 0.444 へ跳ぶ。
なお duty の下限 0.444 は `Disable_duty × BATTERY_VOLTAGE × T_ref_coe` であり、
**0.7 → 0.5 の係数変更では変わらない**（変わるのはスティック位置だけ）。

### T_max は未検証。線形推力仮定の方が疑わしい

`T_max = 2.04 N` は `nmpc_params.hpp` に「暫定値」と明記されており、
**実測ではもっと大きい可能性が高い**との指摘を受けた。

しかし T_max を大きくすると `hover_duty = mass·g/(6·T_max)` は**小さく**なり、
duty 下限 0.444 との乖離は**広がる**。したがって
「T_max が過大」という M2 の推論は成り立たない。

残る説明は**推力カーブが線形でない**こと。プロペラ推力は回転数の 2 乗に比例し、
duty→回転数はほぼ線形なので、実機は 2 乗に近いはずである。
`thrust_exponent = 2.0` なら duty 0.444 での推力は `T_max × 0.197` となり、
桁が合う。

→ `PlantParams` に `thrust_exponent` を追加した。
**既定は 1.0（`nmpc_model.cpp` の線形仮定と一致）**のままにし、
変えるときは理由を残すこととした。実機データが出るまで勝手に動かさない。

### ログ形式は Parquet をやめて npz にした

計画書では Parquet としていたが、この環境の venv に **pandas も pyarrow も無い**。
依存を増やすより、numpy だけで読める `.npz`（圧縮列指向）を選んだ。
実測で **CSV の 1/8**（165 KB vs 1374 KB）に収まり、実用上の不足はない。

### 図の日本語が豆腐になった

matplotlib の既定 DejaVu Sans に CJK が無く、全ラベルが豆腐になった。
ラベルを英語に逃がすとリポジトリの記法（日本語）から外れるので、
`Hiragino Sans` を優先探索する形にした。見つからない環境では警告を出して続行する。

## 対応

- `host/firmware_consts.py`: `control.cpp` / `radio.hpp` の定数を集約。**写しであることを明記。**
- `host/logger.py`: 列指向レコーダ。`runs/run_YYYYMMDD_HHMMSS/` に
  `data.npz` + `meta.json`（+任意で CSV）。meta には**ファームと SIL 双方の git hash**、
  機体パラメータ、ファーム定数を残す。
- `host/plots.py`: overview（姿勢/EKF誤差/角速度/duty/推力）、飽和率、3D 軌跡、数値サマリ。
- `host/run_m3_demo.py`: ステップ・複合舵・突風・スロットル掃引の一連飛行。
- `host/test_m3_logging.py`: 記録の完全性・往復・meta・描画の検証。

## 検証結果

**M3 全項目 PASS。**

| 項目 | 結果 |
|---|---|
| 行数 = tick 数 | 8100 行 / 期待 8100（取りこぼしなし） |
| 記録レート | 400.0 Hz |
| 列数 | 44 |
| 往復の一致 | float32 で完全一致 |
| npz vs CSV | 165 KB vs 1374 KB（**8.3 倍圧縮**） |
| ファーム git hash | `ee68c78d0afa` を記録 |
| 日本語フォント | `Hiragino Sans` |

デモ飛行 56 秒 / 22500 行でも取りこぼしなし。EKF 誤差は平均 0.03 deg。

**未検証**: `T_max` と `thrust_exponent` の実測値。ESC 遅れ・電圧降下（`motor_tau=0`）。

## 次に触る人へ

- ⚠ **`firmware_consts.py` は `control.cpp` の写し。** 実機を変えたらここも直すこと。
  乖離したまま気づかないと、SIL の結論が全部ずれる。
- ⚠ **Python は `NMPC_simulation/.venv` を使う。** システムの python3 に numpy が無い。
- 高度は**開ループ積分**なので `T_max` 較正前は発散する（デモで 10 km まで上がる）。
  図には注意書きを出すようにしてあるが、**制御には一切フィードバックしていない**ので
  姿勢の評価には影響しない。
- 実機のホバー duty を測ったら、`T_max` と `thrust_exponent` の**両方**を決めること。
  片方だけでは決まらない（2 点以上の duty-推力データが要る）。
- 関連: [20260825_sil_m2_closed_loop.md](20260825_sil_m2_closed_loop.md)
