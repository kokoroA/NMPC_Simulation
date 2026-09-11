#ifndef BOARD_LED_HPP
#define BOARD_LED_HPP

#include <stdint.h>

int board_led_init(void);
void board_led_request(uint8_t on);
void board_led_poll(void);

#endif
