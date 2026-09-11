# Benzen_NMPC レビュー: フェイルセーフ不在・OverG が発火不能・T_max 未整合

- **日付**: 2026-08-28
- **種別**: investigation
- **対象**: `Benzen_NMPC/radio.cpp`, `control.cpp`, `sensor.cpp`, `nmpc_params.hpp`, `nmpc.cpp`, `pico_copter.cpp`
- **タグ**: #Benzen_NMPC #failsafe #SBUS #OverG #LSM9DS1 #T_max #ADMM #ESC_calib #ヒステリシス #pico2_w

## 背景 / 症状

ESC キャリブレーション後、実機で全 6 モータが正常に回ることを確認できた
（`20260828_benzen_new_pico2_motor_led.md` の「出力下限 1108 µs が ESC 起動しきい値の真上」仮説の裏付け）。
そこで飛行前に `Benzen_NMPC` 全体をレビューした。

## 原因 / 判断根拠

**確認済み事実。**

- **受信機フェイルセーフが無い。** `radio.cpp` は SBUS フラグバイトを `Chdata[16] = sbus_data[23]` として
  取得しているが、**どこからも参照されていない**。受信途絶のタイムアウトも無い。
  リンク断で `Chdata[]` が最後の値のまま固まり、モータは回り続ける。
- **OverG 保護が数値的に発火不能。** 閾値 `acc_norm > 800.0` は m/s² で約 81.5 g。
  `sensor.cpp:79` は `LSM9DS1_16g`（最大 157 m/s²、3 軸合成でも約 272 m/s²）。**到達し得ない。**
- **`ESC_calib` の二重設定。** `pwm.cpp:7` が 1、`pico_copter.cpp:55` が 0。
  焼かれている `build_fresh/pico_copter.elf` の `main()` を逆アセンブルし、
  `strb r2,[r3]`（r2=0, r3=0x20002920=`ESC_calib`）→ `bl pwm_init` を確認。
  **このファームは校正を実行できない。** 今回成功した校正は ESC 側に残っていた値と考えられる。**未確認。**
- **`T_max` とモデルの整合が取れていない。** `B_c ∝ T_max`。モデル上のホバー duty は
  `T_hover_per_motor / T_max = 0.5821 / 2.04 = 0.285`。実機のホバー duty は未測定。
  なお `nmpc_params.hpp:41` のコメント `// ≈ 0.4905` は誤り（`0.356*9.81/6 = 0.5821`。mass=0.30 kg 時代の残骸）。
- **ADMM の box 下限と実機クランプの不一致。** ソルバは `duty ∈ [0, 0.95]`、
  実機は `minimum_duty = Disable_duty = 0.1` で下から丸める。`duty < 0.1` はモデル外。
- **ゲートにヒステリシスが無い。** `Flight_duty=0.05` / `Disable_duty=0.1` 近傍で
  warm start 破棄と姿勢ゼロ点再取得が繰り返される。離陸直前がこの領域。

**良好だった点（確認済み）。**

- リアルタイム経路（400 Hz IRQ / 100 Hz core1）に**有効な `printf` は 0 個**。
  `Benzen_new` に残る 2 つのような退行は無い。
- `build_fresh`（15:49）に本日のスロットル修正が反映済み（ELF バイト走査で確認）。
- SBUS パーサはバッファ範囲内（`sbus_data[25]`、書き込み 0..24、25 でリセット）。

## 対応

レビュー資料 [`docs/20260828_benzen_nmpc_review.md`](../../20260828_benzen_nmpc_review.md) を作成。
**コード修正は未適用**（ユーザ判断待ち）。着手順は S1 フェイルセーフ → S3 `ESC_calib` 一本化
→ S2 OverG 閾値 → C1 `T_max` 実測。

**却下した案**: OverG 閾値をいきなり 100 m/s² 程度へ下げる。
誤検知で飛行中にモータが止まる方が危険なので、**先に係留ログで分布を見る**。

## 検証結果

- 上記の静的確認（grep / 逆アセンブル / ELF バイト走査 / 単位計算）は**すべて実施済み**。
- **実機での検証は一切していない。** ホバー duty の実測、`T_max` の逆算、
  フェイルセーフの動作確認、core1 スタック高水位の測定はすべて**未実施**。
- `NMPC_BENCH_COST=0` による `iter_us` 短縮量も**未測定**。

## 次に触る人へ

- **フラグやセンサ値を「取得しているが使っていない」箇所に注意。** `Chdata[16]` はその典型で、
  grep すると存在するので実装済みに見える。**代入だけでなく参照があるかを確認すること。**
- **閾値はセンサのフルスケールと突き合わせて検算する。** OverG の 800 m/s² は、
  レンジ設定（`LSM9DS1_16g`）を見れば一目で不能と分かる。単位（mg / m/s² / g）の混同が原因と思われる。
- `ESC_calib` の二重設定は `Benzen_new` / `Benzen_Perfect2025_v1` にも同じ形で存在する。
  **3 つとも直すこと。**
- 関連: [`20260828_benzen_new_pico2_motor_led.md`](20260828_benzen_new_pico2_motor_led.md)、
  [`20260828_nmpc_bench_define_throttle.md`](20260828_nmpc_bench_define_throttle.md)
