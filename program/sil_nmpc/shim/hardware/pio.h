// SIL shim: hardware/pio.h（WS2812 RGB LED。SIL では色を保持するだけ）
#ifndef _HARDWARE_PIO_H
#define _HARDWARE_PIO_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct pio_hw *PIO;
extern PIO pio0;
extern PIO pio1;
uint pio_add_program(PIO pio, const void *program);
uint pio_claim_unused_sm(PIO pio, bool required);
void pio_sm_set_enabled(PIO pio, uint sm, bool enabled);
void pio_sm_put_blocking(PIO pio, uint sm, uint32_t data);
void pio_gpio_init(PIO pio, uint pin);
#ifdef __cplusplus
}
#endif
#endif
