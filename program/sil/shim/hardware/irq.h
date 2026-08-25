// SIL shim: hardware/irq.h
// PWM_IRQ_WRAP に登録されたハンドラ（= loop_400Hz）を SIL スケジューラが直接呼ぶ。
#ifndef _HARDWARE_IRQ_H
#define _HARDWARE_IRQ_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef void (*irq_handler_t)(void);
void irq_set_exclusive_handler(uint num, irq_handler_t handler);
void irq_set_enabled(uint num, bool enabled);
void irq_remove_handler(uint num, irq_handler_t handler);

// SIL 専用: 登録済みハンドラを取得して呼ぶ
irq_handler_t sil_irq_get_handler(uint num);
#ifdef __cplusplus
}
#endif
#endif
