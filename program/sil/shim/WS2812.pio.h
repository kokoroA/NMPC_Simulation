// SIL shim: pico_generate_pio_header が生成するヘッダの代替。
// SIL では PIO を動かさないので、シンボルだけ提供する。
#ifndef _WS2812_PIO_H_SIL_
#define _WS2812_PIO_H_SIL_
#include "hardware/pio.h"
static const struct { int dummy; } ws2812_program = {0};
static inline void ws2812_program_init(PIO, unsigned, unsigned, unsigned, float, unsigned) {}
#endif
