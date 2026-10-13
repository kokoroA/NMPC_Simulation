# `-DNMPC_BENCH_LOG=0` が `#ifdef` で真になり、実機ファームに机上検証分岐が混入していた

- **日付**: 2026-08-28
- **種別**: bugfix | investigation
- **対象**: `Benzen_NMPC/CMakeLists.txt`, `Benzen_NMPC/control.cpp`, `Benzen_NMPC/nmpc_params.hpp`, `Benzen_NMPC/nmpc.cpp`
- **タグ**: #NMPC #pico #RP2350 #PWM #ESC #duty #throttle #ifdef #bench #E9 #74LVC245 #GOKU_HEX #F1404

## 背景 / 症状

`drone_board_hexa_74HC245` + GOKU HEX 13A + T-Motor F1404 KV2900 での実機動作試験。

- `Benzen_NMPC`: スロットルスティックをどう動かしてもモータが一定回転のまま。GP3 と "GP26" の 2 個は回らない。
- `Benzen_new`: スロットルに応じて回転数が正しく変化した。低スロットル域では同じ 2 個が回らず、しきい値を超えると回り始めた。

## 原因 / 判断根拠

**確認済み事実。**

`CMakeLists.txt` は `NMPC_BENCH_LOG=0` / `NMPC_TR1_VERIFY=0` を渡しているが、
コード側は `#ifdef NMPC_BENCH_LOG` で受けている。`#ifdef` は値を見ないので **常に真**。
同ファイルの `DEBUG_SKIP_LOCK_COM` だけは `#if` で正しく書かれており、書き分けの事故である。

結果として実機ビルドに机上検証分岐が入り、
`T_ref = 0.2f * BATTERY_VOLTAGE * ...`（正規は 0.7f）となっていた。
`rate_control()` の停止判定 `T_ref / BATTERY_VOLTAGE < Disable_duty(0.1)` は
`0.2·r < 0.1` すなわち **スティック 50 % 未満で全モータ `motor_stop()`** を意味する。
`set_duty_*()` は `duty<0.01` を 0.01 に持ち上げるので、`motor_stop()` の実出力は **1108 µs 固定**。
これが「いくら動かしても一定回転」の正体。

**実 ELF で裏を取った**（推測ではない）: `build_fresh/pico_copter.elf` に
double `1.4800000220537186`（= `0.2f*7.4`）が 1 個存在し、`5.179999911785126`（= `0.7f*7.4`）は 0 個。

`Benzen_new` は `T_ref = 0.5*V·r`, `T_ref_coe=0.6` なので共通 duty は `2.22·r`。
r=0.046 から 1278 µs → 1860 µs を連続に掃く。**観測どおり正常に動く。**

GP3 / "GP26" だけ回らない件は**別原因**。両ファームで再現しているため出力レンジ側の問題で、
PWM 下限 1108 µs が ESC の起動しきい値の真上にあることが要因。
ESC スロットルキャリブレーション未実施が第一容疑。**未検証。**

## 対応

分析資料 [`docs/20260828_hexa_motor_test_analysis.md`](../../20260828_hexa_motor_test_analysis.md) と
実装計画書 [`docs/20260828_nmpc_throttle_fix_plan.md`](../../20260828_nmpc_throttle_fix_plan.md) を作成し、
**2026-08-28 に修正を適用済み**。内容は以下 3 点。

1. `#ifdef NMPC_BENCH_LOG` / `#ifdef NMPC_TR1_VERIFY` → `#if` に統一
   （未定義マクロは `#if` で 0 と評価されるので SIL ビルドも壊れない）。
   さらに **1 つのマクロが「計装」と「挙動の書き換え」を兼ねていた**ため 3 分割した:
   `NMPC_BENCH_MODE`（挙動）/ `NMPC_BENCH_LOG`（計装）/ `NMPC_BENCH_COST`（`evaluate_cost` 実行）。
   一括で 0 にすると `iter_us` / `loop_us` のテレメトリまで消え、実機の計算時間実測ができなくなるため。
2. スロットル定数を `Benzen_new` に合わせる（`0.5f` / `T_ref_coe=0.6`）
3. 起動判定を `T_ref * T_ref_coe`（= duty スケール）へ統一。
   **`Benzen_new` には既にこの修正を入れたが、`Benzen_NMPC` へ横展開されていなかった。**

**採用しなかった案**: 「`NMPC_BENCH_LOG` の定義自体を CMake から消す」。
値で ON/OFF を切り替えたい意図が明確なので、受け側を `#if` に直すほうが再発しない。

## 検証結果

**静的検証（すべて実施済み・合格）**

- 修正前 ELF に `0.2f*7.4 = 1.4800000220537186` が 1 個、`0.7f*7.4` が 0 個 → **bench 分岐の混入を確定**。
- 修正後 `build_verify/pico_copter.elf`: `0.2f*7.4` = **0 個**、`0.5f*7.4 = 3.7` = **1 個**、
  `T_ref_coe` は float `0.6` = 1 個 / `0.7` = 0 個。**意図した分岐と係数になっている。**
- `static_assert(nmpc::rti::reset_period == 100)` が `-DNMPC_TR1_VERIFY=0` でも**マクロ未定義でも**成立
  → SIL など `-D` を渡さないビルドでも安全。
- 飛行ビルド（`NMPC_BENCH_MODE=0 / LOG=1 / COST=0`）: ビルド成功。
  `debug_print_nmpc_bench()` と `wifi_telemetry_send_bench()` が ELF に残存 → **テレメトリは生きている**。
- 机上ビルド（`MODE=1 / LOG=1 / COST=1`）も**ビルド成功**し、`0.2f*7.4` が 1 個
  → 机上分岐が正しく選ばれる。ガード分割で bench 経路を壊していない。
- `text 674936 → 674216`（-720 B、`evaluate_cost` の除去）、`bss 402456` 変化なし。**メモリ退行なし。**
- `control.cpp` は CRLF。バイナリモードで編集し、行末・行数（2579 → 2582、追加 3 行のみ）を維持。

**未検証（実機が必要）**

- オシロによるパルス幅の実測（0→100 % 掃引で 1108 → 1860 µs を連続に掃くこと）。**未実施。**
- `Benzen_new` と同一スティック位置で同一パルス幅になることの確認。**未実施。**
- `NMPC_BENCH_COST=0` による `iter_us` の短縮量。**未測定**（概算 0.15–0.3 ms、要実測）。
- `T_ref_coe` 0.7 → 0.6 による姿勢制御の権限配分の変化。**未評価。係留確認が必要。**
- 原因 B（GP3 / GP20 が低スロットルで回らない件）は**本修正の対象外**。未着手。

## 次に触る人へ

- **`#ifdef` と `-DX=0` の組み合わせは、このリポジトリで既に 1 回事故を起こしている。**
  CMake の `target_compile_definitions` に `X=0` を書いたら、受け側が `#if` か必ず確認すること。
- `nmpc_params.hpp:99` の `NMPC_TR1_VERIFY` 誤爆により `reset_period` が 100 → 100000 になっていた。
  TR1/SR2 の定期フル再構築が事実上無効。**長時間飛行での解の劣化は未評価。**
- 基板データ上、`SN74LVC245APWR` の `/OE`（pin19）が R3 10k で **+3V3 にプルアップ**されている。
  データどおりなら出力 Hi-Z。実機は動いているので実際は Low のはずだが、**未実測**。
  R3 未実装でフローティングなら不安定要因になる。
- 基板コネクタ `GP4` / `GP5` は実体が **GP14 / GP15**。ESC1..ESC6 の番号は回路図と基板で逆順。
  **モータの議論は `GPxx` リファレンス名で行うこと。**
- Pico に GP26 は使われていない。**物理ピン 26 = GP20**（回路図ネット名 `Net-(U1-Pad26)`）。
- ⚠ **`Benzen_NMPC` は git にまったく入っていない**（`pico-sdk` リポジトリで `?? Benzen_NMPC/`、
  ディレクトリ自身の `.git` も無い）。**サブモジュールのコミットハッシュは残せない。**
  実装計画書には「サブモジュール側のコミットハッシュを記す」と書いたが、**前提が誤っていた**。
  代わりに 2026-08-28 修正後のファイル SHA-256 を記録する:
  - `control.cpp`     `37cca7d1715a04b59c16e357d4172f27b3eeef334fdbf501b85c7602d5dddb49`
  - `nmpc.cpp`        `18b064c2a124388d4189acea8ea2b7bbb8aa02619218d213d5b5b08a051e4d0a`
  - `nmpc_params.hpp` `88a95c38d2c6ec5629130ecc6ff9df0f6c217bcd63a3fdcc23868ab8ed6201cc`
  - `CMakeLists.txt`  `03048571d8f654aa27f09099bec89e220f14d7b8587cba5585f8066cfcd58778`
  **実機ファームの版管理をどうするかは未解決の課題である。**
- 修正前のファームは `build_fresh/pico_copter.uf2`（2026-08-28 12:23）に残っている。
  **bench 誤爆版なので比較用途以外で通電しないこと。**
- 関連: [`20260820_hardware_phase_correction.md`](20260820_hardware_phase_correction.md)
