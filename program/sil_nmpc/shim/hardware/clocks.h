#ifndef _HARDWARE_CLOCKS_H
#define _HARDWARE_CLOCKS_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
enum clock_index { clk_sys = 0, clk_peri = 1 };
uint32_t clock_get_hz(enum clock_index clk_index);
bool set_sys_clock_khz(uint32_t khz, bool required);
#ifdef __cplusplus
}
#endif
#endif
