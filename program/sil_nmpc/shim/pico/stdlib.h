// SIL shim: pico/stdlib.h
// 実機の pico-sdk を置き換える。時間は「仮想時間」で、SIL スケジューラが進める。
#ifndef _PICO_STDLIB_H
#define _PICO_STDLIB_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif

#define PICO_OK 0
#define PICO_ERROR_GENERIC (-1)

// --- 仮想時間 -------------------------------------------------------------
uint32_t time_us_32(void);
uint64_t time_us_64(void);
void     sleep_ms(uint32_t ms);
void     sleep_us(uint64_t us);
static inline void tight_loop_contents(void) {}

// SIL 専用: 時間を進める（ホスト側から呼ぶ）
void sil_time_advance_us(uint64_t us);

// --- stdio ----------------------------------------------------------------
int  stdio_init_all(void);

// --- GPIO（すべて no-op、状態のみ保持） -----------------------------------
#define GPIO_OUT 1
#define GPIO_IN  0
#define GPIO_FUNC_SPI  1
#define GPIO_FUNC_UART 2
#define GPIO_FUNC_I2C  3
#define GPIO_FUNC_PWM  4
#define GPIO_FUNC_SIO  5
#define GPIO_FUNC_PIO0 6
#define GPIO_FUNC_PIO1 7
void gpio_init(uint gpio);
void gpio_set_dir(uint gpio, bool out);
void gpio_put(uint gpio, bool value);
bool gpio_get(uint gpio);
void gpio_set_function(uint gpio, int fn);
void gpio_set_pulls(uint gpio, bool up, bool down);
void gpio_set_input_enabled(uint gpio, bool enabled);
void gpio_pull_up(uint gpio);
void gpio_disable_pulls(uint gpio);

#ifdef __cplusplus
}
#endif
#endif
