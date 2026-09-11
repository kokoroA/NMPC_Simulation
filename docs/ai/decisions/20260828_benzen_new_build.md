# Benzen_new のビルドコマンドを確定した

- **日付**: 2026-08-28
- **種別**: investigation
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_new/`
- **タグ**: #Benzen_new #Pico #pico2_w #RP2350 #cmake #picotool

## 背景 / 症状

PID 実機 `Benzen_new` の cmake 手順がリポジトリ内にまとまっていない。`README.md` は「# PicoCopter」のみ。
成功例は `20260825_benzen_new_pico2w.md` / `20260824_benzen_new_auto_maneuvers.md` の「`build_fresh` で成功」だけで、コマンド自体は残っていなかった。
同日の `20260828_benzen_nmpc_build.md` が NMPC 側の正。こちらは PID 側の鏡像。

## 原因 / 判断根拠

- プロジェクトは pico-sdk サブモジュール配下の外部 CMake。`pico_sdk_import.cmake` は `PICO_SDK_PATH` 必須。
- `CMakeLists.txt` が `PICO_BOARD pico2_w` を FORCE。コマンドラインでボードを指定する必要はない。
- PATH に picotool が無く、初回 cmake が 2.1.1 をソース取得する（NMPC と同じ）。
- 既存 `build/` は 2024-09-20・`PICO_BOARD=pico`（RP2040, family `0xe48bff56`）。Pico 2 W には入れない。
- `build_fresh` は 2026-08-26 の Pico 2 W 成果物。cache の SDK パスは現行 `research_project/.../pico-sdk`。
- Eigen の `include_directories` は CMakeLists にあるが、ソースは Eigen を `#include` していない（NMPC コピーの名残）。ビルド成否は Eigen パスに依存しない。未検証: パス欠落時に cmake が失敗するか。
- `CMAKE_C_STNDARD` は typo（`STANDARD` ではない）。C 標準は実質未設定。実害は未検証。

却下: 既存 `build/` を消して作り直す案（RP2040 旧成果物の参照を失う。Pico 2 W 用ではないので上書きも無意味）。

## 対応

推奨コマンド（`build_fresh` の CMakeCache と一致。再ビルドは本調査では未実施）:

```bash
export PICO_SDK_PATH="/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk"
cd "$PICO_SDK_PATH/Benzen_new"
cmake -S . -B build_fresh -DPICO_SDK_PATH="$PICO_SDK_PATH"
cmake --build build_fresh --target pico_copter -j8
```

成果物: `Benzen_new/build_fresh/pico_copter.uf2`

書き込みは NMPC と同じ（BOOTSEL → `/Volumes/RP2350/`、または `build_fresh/_deps/picotool-build/picotool load -f`）。

ホスト境界テスト（pico-sdk 不要）:

```bash
c++ -std=c++17 -O2 auto_maneuvers.cpp auto_maneuvers_test.cpp -o auto_maneuvers_test
./auto_maneuvers_test
```

（コマンドは推定。ディレクトリ直下のバイナリ `auto_maneuvers_test` は 2026-08-25 付。CMake ターゲットではない。）

SIL（ホスト共有ライブラリ、UF2 ではない）は `program/sil/USAGE.md`。既定 `FW_DIR` がこの `Benzen_new`。

## 検証結果

- 既存 `build_fresh/pico_copter.elf`（2026-08-26 14:59）: `PICO_BOARD=pico2_w`, `PICO_PLATFORM=rp2350-arm-s`, Release。
- UF2 family `0xe48bff57`（RP2350-ARM-S）、サイズ 1,306,112 B。
- `arm-none-eabi-size`: text 652792 / bss **236772**（SRAM 520 KB の約 44.5%。`20260825_sil_m0_firmware_on_host.md` と一致）。
- `pwm.cpp` は 2026-08-28 15:51 更新。現行 `build_fresh` の UF2 はそれより古い。ソース変更後は再ビルドが必要。
- 本調査では cmake 再実行していない（未検証: 今日のソースで通るか）。
- 実機書き込み・飛行は未検証。

## 次に触る人へ

- 書く UF2 は `Benzen_new/build_fresh/pico_copter.uf2`。Pico 1 には入れない（Pico 1 は `Benzen_pico1`）。
- `build/` は RP2040 旧バイナリ。Pico 2 W 用に使わない。消さなくてもよいが、混ぜない。
- Wi-Fi AP は SSID `Benzen_new` / パスワード `benzen2026`。手順は `Benzen_new/doc/wifi_telemetry.md`。
- NMPC と同時に PC へ書くと SSID が違うので衝突しない。
