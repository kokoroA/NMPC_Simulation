// SIL shim: hardware/pwm.h
// set_duty_* が書き込む PWM チャンネルレベルを保持し、SIL が duty として読み出す。
#ifndef _HARDWARE_PWM_H
#define _HARDWARE_PWM_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
#define PWM_CHAN_A 0
#define PWM_CHAN_B 1
#define PWM_IRQ_WRAP 4

uint  pwm_gpio_to_slice_num(uint gpio);
uint  pwm_gpio_to_channel(uint gpio);
void  pwm_set_wrap(uint slice_num, uint16_t wrap);
void  pwm_set_clkdiv(uint slice_num, float divider);
void  pwm_set_chan_level(uint slice_num, uint chan, uint16_t level);
void  pwm_set_enabled(uint slice_num, bool enabled);
void  pwm_clear_irq(uint slice_num);
void  pwm_set_irq_enabled(uint slice_num, bool enabled);

// SIL 専用: slice/chan から生の level を読む（HAL が duty へ戻す）
uint16_t sil_pwm_get_level(uint slice_num, uint chan);
#ifdef __cplusplus
}
#endif
#endif
