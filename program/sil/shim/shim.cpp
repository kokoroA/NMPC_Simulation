// SIL shim 実体
// 方針: 実機の pico-sdk API を「仮想時間」と「状態保持」で置き換える。
//       制御ロジック側のコードは一切変更しない。
#include "pico/stdlib.h"
#include "pico/sync.h"
#include "pico/multicore.h"
#include "pico/cyw43_arch.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "hardware/spi.h"
#include "hardware/i2c.h"
#include "hardware/uart.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

#include <atomic>
#include <condition_variable>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <thread>

// =========================================================================
// 仮想時間
//   実機は自走クロックだが、SIL ではホストが明示的に進める。
//   これにより「同じ入力 → 同じログ」の決定性が保証される。
// =========================================================================
namespace {
std::atomic<uint64_t> g_time_us{0};
}

extern "C" uint32_t time_us_32(void) { return (uint32_t)g_time_us.load(); }
extern "C" uint64_t time_us_64(void) { return g_time_us.load(); }
extern "C" void sil_time_advance_us(uint64_t us) { g_time_us.fetch_add(us); }

// sleep は仮想時間を進めるだけ（ホストは待たない）
extern "C" void sleep_ms(uint32_t ms) { g_time_us.fetch_add((uint64_t)ms * 1000ull); }
extern "C" void sleep_us(uint64_t us) { g_time_us.fetch_add(us); }

extern "C" int stdio_init_all(void) { return 1; }

// =========================================================================
// GPIO（状態のみ保持）
// =========================================================================
namespace {
bool g_gpio_state[64] = {false};
}
extern "C" {
void gpio_init(uint) {}
void gpio_set_dir(uint, bool) {}
void gpio_put(uint gpio, bool v) { if (gpio < 64) g_gpio_state[gpio] = v; }
bool gpio_get(uint gpio) { return gpio < 64 ? g_gpio_state[gpio] : false; }
void gpio_set_function(uint, int) {}
void gpio_set_pulls(uint, bool, bool) {}
void gpio_set_input_enabled(uint, bool) {}
void gpio_pull_up(uint) {}
void gpio_disable_pulls(uint) {}
}

// =========================================================================
// PWM: set_duty_* が書いた level を保持する
// =========================================================================
namespace {
constexpr int kMaxSlice = 12;
uint16_t g_pwm_level[kMaxSlice][2] = {{0}};
uint16_t g_pwm_wrap[kMaxSlice] = {0};
}
extern "C" {
uint pwm_gpio_to_slice_num(uint gpio) { return (gpio >> 1) & 0x7; }
uint pwm_gpio_to_channel(uint gpio) { return gpio & 1; }
void pwm_set_wrap(uint s, uint16_t w) { if (s < kMaxSlice) g_pwm_wrap[s] = w; }
void pwm_set_clkdiv(uint, float) {}
void pwm_set_chan_level(uint s, uint c, uint16_t l) { if (s < kMaxSlice && c < 2) g_pwm_level[s][c] = l; }
void pwm_set_enabled(uint, bool) {}
void pwm_clear_irq(uint) {}
void pwm_set_irq_enabled(uint, bool) {}
uint16_t sil_pwm_get_level(uint s, uint c) { return (s < kMaxSlice && c < 2) ? g_pwm_level[s][c] : 0; }
}

// =========================================================================
// IRQ: PWM_IRQ_WRAP ハンドラ（= loop_400Hz）を保持し、SIL から直接呼ぶ
// =========================================================================
namespace {
constexpr int kMaxIrq = 32;
irq_handler_t g_irq_handler[kMaxIrq] = {nullptr};
}
extern "C" {
void irq_set_exclusive_handler(uint n, irq_handler_t h) { if (n < kMaxIrq) g_irq_handler[n] = h; }
void irq_set_enabled(uint, bool) {}
void irq_remove_handler(uint n, irq_handler_t) { if (n < kMaxIrq) g_irq_handler[n] = nullptr; }
irq_handler_t sil_irq_get_handler(uint n) { return n < kMaxIrq ? g_irq_handler[n] : nullptr; }
}

// =========================================================================
// semaphore: core0 → core1 の 100 Hz 起動を再現
//   決定性のため「core1 が次の acquire に戻るまで待つ」ハンドシェイクを持つ。
// =========================================================================
namespace {
struct SemImpl {
    std::mutex m;
    std::condition_variable cv;
    int permits = 0;
    // core1 が acquire 待ちに入っているか
    std::atomic<bool> waiting{false};
};
SemImpl g_sem_pool[4];
int g_sem_used = 0;

SemImpl *impl_of(semaphore_t *s) { return static_cast<SemImpl *>(s->impl); }
}

extern "C" {
void sem_init(semaphore_t *sem, int16_t initial, int16_t) {
    SemImpl *im = &g_sem_pool[g_sem_used < 4 ? g_sem_used++ : 3];
    im->permits = initial;
    sem->impl = im;
}
void sem_release(semaphore_t *sem) {
    SemImpl *im = impl_of(sem);
    { std::lock_guard<std::mutex> lk(im->m); im->permits++; }
    im->cv.notify_one();
}
void sem_reset(semaphore_t *sem, int16_t permits) {
    SemImpl *im = impl_of(sem);
    std::lock_guard<std::mutex> lk(im->m);
    im->permits = permits;
}
void sem_acquire_blocking(semaphore_t *sem) {
    SemImpl *im = impl_of(sem);
    std::unique_lock<std::mutex> lk(im->m);
    im->waiting.store(true);
    im->cv.wait(lk, [im] { return im->permits > 0; });
    im->waiting.store(false);
    im->permits--;
}
bool sem_try_acquire(semaphore_t *sem) {
    SemImpl *im = impl_of(sem);
    std::lock_guard<std::mutex> lk(im->m);
    if (im->permits > 0) { im->permits--; return true; }
    return false;
}
}

// =========================================================================
// multicore: core1 をホストスレッドとして起動
// =========================================================================
namespace {
std::thread g_core1;
std::atomic<bool> g_core1_running{false};
}
extern "C" {
void multicore_launch_core1(void (*entry)(void)) {
    if (g_core1_running.exchange(true)) return;
    g_core1 = std::thread([entry] { entry(); });
    g_core1.detach();
}
void multicore_reset_core1(void) {}

// core1 が sem 待ちに戻るまでブロック（1 tick の完了を保証する）
void sil_core1_wait_idle(void) {
    if (!g_core1_running.load()) return;
    for (int i = 0; i < 4; ++i) {
        SemImpl *im = &g_sem_pool[i];
        if (im->waiting.load()) return;
    }
    // まだ走っている間は譲る
    for (int spin = 0; spin < 100000; ++spin) {
        for (int i = 0; i < 4; ++i)
            if (g_sem_pool[i].waiting.load()) return;
        std::this_thread::yield();
    }
}
}

// =========================================================================
// CYW43 / SPI / I2C / UART / PIO: すべて no-op
// =========================================================================
extern "C" {
int  cyw43_arch_init(void) { return 0; }
void cyw43_arch_deinit(void) {}
void cyw43_arch_gpio_put(int, bool) {}
bool cyw43_arch_gpio_get(int) { return false; }

spi_inst_t *spi0 = reinterpret_cast<spi_inst_t *>(1);
spi_inst_t *spi1 = reinterpret_cast<spi_inst_t *>(2);
uint spi_init(spi_inst_t *, uint b) { return b; }
void spi_set_format(spi_inst_t *, uint, uint, uint, uint) {}
int  spi_write_blocking(spi_inst_t *, const uint8_t *, size_t len) { return (int)len; }
int  spi_read_blocking(spi_inst_t *, uint8_t, uint8_t *dst, size_t len) { std::memset(dst, 0, len); return (int)len; }
int  spi_write_read_blocking(spi_inst_t *, const uint8_t *, uint8_t *dst, size_t len) { std::memset(dst, 0, len); return (int)len; }

i2c_inst_t *i2c0 = reinterpret_cast<i2c_inst_t *>(1);
i2c_inst_t *i2c1 = reinterpret_cast<i2c_inst_t *>(2);
uint i2c_init(i2c_inst_t *, uint b) { return b; }
int  i2c_write_blocking(i2c_inst_t *, uint8_t, const uint8_t *, size_t len, bool) { return (int)len; }
int  i2c_read_blocking(i2c_inst_t *, uint8_t, uint8_t *dst, size_t len, bool) { std::memset(dst, 0, len); return -1; }
int  i2c_write_timeout_us(i2c_inst_t *, uint8_t, const uint8_t *, size_t len, bool, uint) { return (int)len; }
int  i2c_read_timeout_us(i2c_inst_t *, uint8_t, uint8_t *dst, size_t len, bool, uint) { std::memset(dst, 0, len); return -1; }

uart_inst_t *uart0 = reinterpret_cast<uart_inst_t *>(1);
uart_inst_t *uart1 = reinterpret_cast<uart_inst_t *>(2);
uint uart_init(uart_inst_t *, uint b) { return b; }
void uart_deinit(uart_inst_t *) {}
uint uart_set_baudrate(uart_inst_t *, uint b) { return b; }
void uart_set_hw_flow(uart_inst_t *, bool, bool) {}
void uart_set_format(uart_inst_t *, uint, uint, uart_parity_t) {}
void uart_set_fifo_enabled(uart_inst_t *, bool) {}
void uart_set_irq_enables(uart_inst_t *, bool, bool) {}
bool uart_is_readable(uart_inst_t *) { return false; }
bool uart_is_writable(uart_inst_t *) { return true; }
char uart_getc(uart_inst_t *) { return 0; }
void uart_putc(uart_inst_t *, char) {}
void uart_puts(uart_inst_t *, const char *) {}
void uart_write_blocking(uart_inst_t *, const uint8_t *, size_t) {}
void uart_read_blocking(uart_inst_t *, uint8_t *dst, size_t len) { std::memset(dst, 0, len); }

PIO pio0 = reinterpret_cast<PIO>(1);
PIO pio1 = reinterpret_cast<PIO>(2);
uint pio_add_program(PIO, const void *) { return 0; }
uint pio_claim_unused_sm(PIO, bool) { return 0; }
void pio_sm_set_enabled(PIO, uint, bool) {}
void pio_sm_put_blocking(PIO, uint, uint32_t) {}
void pio_gpio_init(PIO, uint) {}

uint32_t clock_get_hz(enum clock_index) { return 125000000u; }
bool set_sys_clock_khz(uint32_t, bool) { return true; }
}
