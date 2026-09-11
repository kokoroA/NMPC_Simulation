
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#define UART_ID uart0
#define BAUD_RATE 100000
#define DATA_BITS 8
#define STOP_BITS 2
#define PARITY UART_PARITY_EVEN
#define CH1MAX 1680
#define CH1MIN 368
#define CH2MAX 1680
#define CH2MIN 368
#define CH3MAX 1638
#define CH3MIN 368
#define CH4MAX 1638
#define CH4MIN 368
#define CH5MAX 1638
#define CH5MIN 368
#define CH6MAX 1638
#define CH6MIN 368
#define SERVO_MIN 144
#define SERVO_MAX 1904

#define MODE_SW 10
#define LANDING_SW 11
#define LOG 6
#define SERVO 4
// #define REDCIRCLE 5
#define FTC 5
#define LINETRACE 7
// #define T_ref_up 7
#define ROCKING 200 //8
#define HOVERING 8

#define UART_ID2 uart1
#define BAUD_RATE2 9600
#define DATA_BITS2 8
#define STOP_BITS2 1
#define PARITY2  UART_PARITY_NONE
#define UART_TX_PIN2 4
#define UART_RX_PIN2 5

//０番と1番ピンに接続
#define UART_TX_PIN 0
#define UART_RX_PIN 1

//グローバル変数の宣言
extern uint16_t Chdata[18];
// 最後に SBUS フレームを完結させた時刻 [us]。受信途絶の検出に使う。
extern volatile uint32_t Radio_last_frame_us;
// SBUS フラグバイト (sbus_data[23]) のビット
#define SBUS_FLAG_FRAME_LOST 0x04
#define SBUS_FLAG_FAILSAFE   0x08
// 受信品質の計装。SBUS フレームが 1 つ完結するたびに radio.cpp が更新する。
// FRAME_LOST(0x04) は 1 フレーム (7-14 ms) 取りこぼしただけで立つ日常的なフラグであり、
// 「操縦不能」を意味しない。単発で判断せず Rx_framelost_run (連続数) で見ること。
// 400Hz 側で数えると 1 フレームを複数回数えてしまうので、必ずフレーム単位で数える。
extern volatile uint8_t  Rx_sbus_flags;       // sbus_data[23] の生値
extern volatile uint8_t  Rx_framelost_run;    // 0x04 が連続しているフレーム数 (255 で飽和)
extern volatile uint32_t Rx_framelost_count;  // 0x04 の累積回数 (診断用)

//グローバル関数の宣言
void radio_init(void);
