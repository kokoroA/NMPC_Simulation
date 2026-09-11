# Benzen_new を Pico 2 W（RP2350）向けに移植

- **日付**: 2026-08-25
- **種別**: design
- **対象**: `program/pico/pico-sdk2/pico-sdk/Benzen_new/`（CMakeLists, pwm, board_led, pico_copter, control）
- **タグ**: #Benzen_new #Pico #pico2_w #RP2350 #E9 #CYW43 #board_led

## 背景 / 症状

`Benzen_new` はボード未指定のまま SDK デフォルト `pico`（RP2040）でビルドされていた。
UF2 family ID は `0xe48bff56`（RP2040）であり、Pico 2 W のブートローダは受け付けない。
`Benzen_NMPC` は既に `PICO_BOARD=pico2_w` で実機動作している。

## 原因 / 判断根拠

- Pico 2 W は RP2350。RP2040 UF2 では起動しない（確認済み: family ID 不一致）。
- GPIO25 は非 W Pico のオンボード LED だが、Pico 2 W では **CYW43 の SPI CS**（`CYW43_DEFAULT_PIN_WL_CS`）。GPIO 直叩きは無線チップと衝突する。
- Pico 2 W の LED は CYW43 上。`cyw43_arch_gpio_put` は PWM IRQ / core1 から呼べないため、`Benzen_NMPC` と同じ request/poll が必要。
- RP2350-E9: PWM ピンの入力バッファを無効化しないと出力が不安定になり得る（`Benzen_NMPC/pwm.cpp` の既存 workaround を踏襲）。
- Pico 2 の既定 SYSCLK は 150 MHz。PWM は `wrap=3124, clkdiv=100` で **125 MHz 前提の 400 Hz**。未固定だと制御周期が約 480 Hz になる。

## 対応

- `PICO_BOARD pico2_w` を CMake で強制。Wi-Fi テレメは入れず `pico_cyw43_arch_none` のみ（LED 用）。
- `SYS_CLK_KHZ=125000` で PWM 400 Hz を維持。
- `board_led` を新設。400 Hz 側は `board_led_request`、main で `board_led_poll`。
- PWM ピンの E9 workaround と `pwm_pins_early_init` を追加。モータ GPIO 割り当ては変更しない。

却下: GPIO25 をそのまま残す案（CS ピンと衝突）。`pico_cyw43_arch_lwip_*` を足す案（飛行に不要）。

## 検証結果

- `build_fresh` で `pico_copter` ビルド成功。
- `PICO_BOARD=pico2_w`, `PICO_PLATFORM=rp2350-arm-s`。
- UF2 family ID `0xe48bff57`（RP2350-ARM-S）。
- 実機書き込み・飛行は未検証。

## 次に触る人へ

- 書き込む UF2 は `Benzen_new/build_fresh/pico_copter.uf2`。
- オンボード LED が点かない場合は `#board led init failed` を USB シリアルで確認（CYW43 初期化失敗）。
- モータ配線は従来の `set_duty_*` マッピングのまま。NMPC 側のコメントと一致しない箇所があるので、こちらを正とすること。
