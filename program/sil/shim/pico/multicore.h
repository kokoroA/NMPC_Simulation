// SIL shim: multicore（core1 をホストスレッドとして起動する）
#ifndef _PICO_MULTICORE_H
#define _PICO_MULTICORE_H
#include "pico/types.h"
#include "pico/sync.h"
#ifdef __cplusplus
extern "C" {
#endif
void multicore_launch_core1(void (*entry)(void));
void multicore_reset_core1(void);

// SIL 専用: core1 が sem 待ちに戻るまでブロックする（決定性の確保）
void sil_core1_wait_idle(void);
#ifdef __cplusplus
}
#endif
#endif
