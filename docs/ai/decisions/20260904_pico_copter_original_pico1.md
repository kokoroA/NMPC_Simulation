# kouhei1970/pico_copter を Pico 1 向けに clone・ビルド

- **日付**: 2026-09-04
- **種別**: design
- **対象**: `program/pico/pico-sdk2/pico-sdk/pico_copter_original/`
- **タグ**: #pico_copter #Pico #RP2040 #clone

## 背景 / 症状

上流 [kouhei1970/pico_copter](https://github.com/kouhei1970/pico_copter.git) を Pico 1（RP2040）向けにビルドする必要があった。
既存 `pico-sdk/pico_copter/` は同一 origin だが、NMPC ソースと `control.cpp` 改変が乗っている。

## 原因 / 判断根拠

- 既存ディレクトリを上書きすると NMPC 改変が消える。
- 上流 CMakeLists の Eigen は `/Users/itoukouhei/pico/eigen` で、このマシンには無い。
- Pico 1 は `PICO_BOARD=pico` / family `0xe48bff56`。未指定だと SDK 既定も pico だが、FORCE して Pico 2 W 成果物と混ぜない。

却下: 既存 `pico_copter/` を reset して上流に戻す案（ローカル NMPC 作業が消える）。

## 対応

- 新規ディレクトリ `pico_copter_original` へ clone。
- `PICO_BOARD pico` を FORCE。Eigen を `/Users/kokoro/Documents/drone_project/pico/eigen` に差し替え。ソースは未改変。

## 検証結果

- `build_fresh` で `pico_copter` ビルド成功。
- `PICO_BOARD=pico`, `PICO_PLATFORM=rp2040`, Release。
- UF2 family ID `0xe48bff56`（RP2040）。
- SRAM: text 389,164 / bss 210,396。RAM overflow なし。
- 実機書き込み・飛行は未検証。

## 次に触る人へ

- 書く UF2 は `pico_copter_original/build_fresh/pico_copter.uf2`。Pico 2 W には入れない。
- 既存 `pico_copter/` は NMPC 改変版。上流の PID ファームはこちら。
- 対象は **非 W の Pico 1**（LED は GPIO25）。
