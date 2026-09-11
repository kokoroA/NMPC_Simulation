# Benzen_NMPC の飛行 LED が一面黄・終端 2 個消灯に見える

- **日付**: 2026-08-31
- **種別**: investigation
- **対象**: `Benzen_NMPC/rgbled.cpp`（対比: `Benzen_Perfect2025/rgbled.cpp`, `Benzen_Perfect2025_v1/rgbled.cpp`）
- **タグ**: #Benzen_NMPC #Benzen_Perfect2025 #LED #WS2812 #rgbled_normal #飛行状態

## 背景 / 症状

実機で `Benzen_NMPC` の飛行状態 LED が `Benzen_Perfect2025` と違う。
終端の LED 2 個が消灯し、色も一面黄色のように見える。

前件 `20260828_benzen_new_pico2_motor_led.md` は **Benzen_new** の交互点滅であり、
今回の「一面黄＋終端 2 消灯」とは別症状。

## 原因 / 判断根拠

**確認済み事実。**

- ドライバ層は同一。`RGBLED_PIN=16`、`RGBLED_LENGTH=36`、`pio0` SM0、`FORMAT_GRB`。
  本数不足や RGB/GRB 取り違えでは説明できない。
- 飛行中（`Arm_flag==2` かつ `Flight_mode==NORMAL`）は両系統とも `rgbled_normal()` を呼ぶ。
- 差は `rgbled_normal()` の **画素 0〜8 の色定義だけ**。

| index | Perfect2025 / v1 | NMPC |
|---|---|---|
| 0–2 | 黄 `(225,135,0)` | 黄 `(225,135,0)` |
| 3–5 | 青 `(0,0,225)` | 黄 `(225,135,0)` |
| 6 | 赤 `(225,0,0)` | 黄 `(225,135,0)` |
| 7–8 | 赤 `(225,0,0)` | **消灯** `(0,0,0)`（コメント: 色境界は付けない） |

NMPC は機首側 7 個を黄ベタにし、その直後 2 個を意図的に消している。
これが「一面黄色」「終端 2 個ついていない」に一致する。

`rgbled.cpp` のタイムスタンプは NMPC が 2024-09-20、Perfect2025 が 2026-08-28。
NMPC は古い区切り塗り（黄/緑/赤/青＋境界消灯）のまま残っており、
Perfect2025 側で後から機首を黄・青・赤の 3 色帯に更新した。**移植漏れ。**

**クロック / PIO / Wi-Fi は犯人ではない。** 同じ WS2812 初期化で Perfect2025 は正しい見た目になる。
前回の「RP2350 の SYSCLK を疑う」は既に却下済み（`20260828_benzen_new_pico2_motor_led.md`）。

**推測（未検証）。** `control.hpp` では `#define REDCIRCLE 3` がコメントアウトされており、
`Flight_mode = REDCIRCLE` は `radio.hpp` のチャンネル番号 `REDCIRCLE=5` に落ちる。
スイッチ位置によっては `rgbled_redcircle()` に入るが、NMPC ではこちらも黄ベタ＋境界消灯なので
見た目はほぼ同じ。症状の主因ではない。

## 対応

コード修正は未適用（分析依頼のため）。直すなら NMPC の `rgbled_normal()` 画素 0〜8 を
Perfect2025 と同じ黄 3 / 青 3 / 赤 3 に揃える。`rgbled_redcircle()` は触らない。

却下: `RGBLED_LENGTH` を増やす、FORMAT を変える、PIO を pio1 に移す。
いずれも症状と一致しない。

## 検証結果

- ソース diff による静的確認のみ。実機再点灯は未実施。

## 次に触る人へ

飛行 LED の見た目は `led_control()` ではなく **`rgbled_normal()` の画素配列** が正。
`LENGTH` や PIO を疑う前に、画素 0〜8 を先に見ること。
