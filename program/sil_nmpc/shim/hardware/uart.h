// SIL shim: hardware/uart.h（RC 受信は Chdata[] 直接注入なので実体不要）
#ifndef _HARDWARE_UART_H
#define _HARDWARE_UART_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct uart_inst uart_inst_t;
extern uart_inst_t *uart0;
extern uart_inst_t *uart1;
typedef enum { UART_PARITY_NONE = 0, UART_PARITY_EVEN, UART_PARITY_ODD } uart_parity_t;
#define UART_IRQ_RX 0
uint uart_init(uart_inst_t *uart, uint baudrate);
void uart_deinit(uart_inst_t *uart);
uint uart_set_baudrate(uart_inst_t *uart, uint baudrate);
void uart_set_hw_flow(uart_inst_t *uart, bool cts, bool rts);
void uart_set_format(uart_inst_t *uart, uint data_bits, uint stop_bits, uart_parity_t parity);
void uart_set_fifo_enabled(uart_inst_t *uart, bool enabled);
void uart_set_irq_enables(uart_inst_t *uart, bool rx_has_data, bool tx_needs_data);
bool uart_is_readable(uart_inst_t *uart);
bool uart_is_writable(uart_inst_t *uart);
char uart_getc(uart_inst_t *uart);
void uart_putc(uart_inst_t *uart, char c);
void uart_puts(uart_inst_t *uart, const char *s);
void uart_write_blocking(uart_inst_t *uart, const uint8_t *src, size_t len);
void uart_read_blocking(uart_inst_t *uart, uint8_t *dst, size_t len);
#define UART0_IRQ 20
#define UART1_IRQ 21
#ifdef __cplusplus
}
#endif
#endif
