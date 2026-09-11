# Benzen_NMPC のビルドコマンドを確定した

- **日付**: 2026-08-28
- **種別**: investigation
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_NMPC/`
- **タグ**: #Benzen_NMPC #Pico #pico2_w #RP2350 #cmake #picotool #Eigen

## 背景 / 症状

ファームのビルド手順がリポジトリ内にまとまっていなかった。`README.md` は「# PicoCopter」のみ。
`NMPC_REPORT.md` / `plan.md` にも cmake コマンドは無い。`20260820_hardware_phase_correction.md` は
「ビルドは未検証」と書いてある。`Benzen_new` 側の `build_fresh` 成功例だけが先行していた。

## 原因 / 判断根拠

- プロジェクトは pico-sdk サブモジュール配下の外部 CMake プロジェクト。SDK 同梱 examples とは別。
- `pico_sdk_import.cmake` は `PICO_SDK_PATH` 必須。未指定だと FATAL_ERROR。
- Eigen は `/Users/kokoro/Documents/drone_project/pico/eigen` の絶対パス。別プロジェクト依存。
- PATH に picotool が無く、初回 cmake が 2.1.1 をソース取得する（`Benzen_new` と同じ）。
- 既存 `build/` は 2026-07-15 の成果物。予稿・SIL の ELF 解析に使っているので上書きしない。

## 対応

手順の正を `docs/benzen_nmpc_build.md` に置いた（親リポジトリ追跡対象）。
`Benzen_NMPC/README.md` からそこへ誘導する。

推奨コマンド（検証済み）:

```bash
cmake -S . -B build_fresh \
  -DPICO_SDK_PATH=/Users/kokoro/Documents/research_project/program/pico/pico-sdk2/pico-sdk
cmake --build build_fresh --target pico_copter -j8
```

却下: 既存 `build/` を消して作り直す案（7/15 ELF を失う）。PATH の picotool 必須化（未インストールが現状）。

## 検証結果

- `build_fresh` で `pico_copter` ビルド成功（構成 約 7 s、コンパイル 約 86 s, `-j8`）。
- `PICO_BOARD=pico2_w`, `PICO_PLATFORM=rp2350-arm-s`, `CMAKE_BUILD_TYPE=Release`。
- UF2 family ID `0xe48bff57`（RP2350-ARM-S）。
- `arm-none-eabi-size`: text 674936 / bss 402456。7/15 ELF の SRAM 75.6% と一致。
- 実機書き込み・飛行は未検証。

## 次に触る人へ

- 書く UF2 は `Benzen_NMPC/build_fresh/pico_copter.uf2`。Pico 1 には入れない。
- Eigen パスを変えたら CMakeLists 23 行目だけ直せばよい。SDK パスは `-D` で足りる。
- `N` の正は `nmpc_params.hpp`（17）。変えたら size と計算時間を取り直す。
