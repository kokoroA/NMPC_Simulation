// SIL shim: CYW43（無線・LED）。SIL では全て no-op。
#ifndef _PICO_CYW43_ARCH_H
#define _PICO_CYW43_ARCH_H
#include "pico/stdlib.h"
#define CYW43_WL_GPIO_LED_PIN 0
#ifdef __cplusplus
extern "C" {
#endif
int  cyw43_arch_init(void);
void cyw43_arch_deinit(void);
void cyw43_arch_gpio_put(int pin, bool value);
bool cyw43_arch_gpio_get(int pin);
#ifdef __cplusplus
}
#endif
#endif
