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

// SIL 専用: 1 tick の完了ハンドシェイク（決定性の確保）
void sil_sched_snapshot(uint64_t *release_count, uint64_t *acquire_entry_count);
void sil_core1_wait_done(uint64_t release_before, uint64_t acquire_entry_before);
#ifdef __cplusplus
}
#endif
#endif
