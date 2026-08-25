// SIL shim: semaphore_t（実機の core0→core1 時間統制を再現）
#ifndef _PICO_SYNC_H
#define _PICO_SYNC_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct semaphore { void *impl; } semaphore_t;
void sem_init(semaphore_t *sem, int16_t initial_permits, int16_t max_permits);
void sem_release(semaphore_t *sem);
void sem_reset(semaphore_t *sem, int16_t permits);
void sem_acquire_blocking(semaphore_t *sem);
bool sem_try_acquire(semaphore_t *sem);
#ifdef __cplusplus
}
#endif
#endif
