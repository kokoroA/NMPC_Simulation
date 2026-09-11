#include "board_led.hpp"
#include "pico/stdlib.h"

#ifdef CYW43_WL_GPIO_LED_PIN
#include "pico/cyw43_arch.h"
#endif

static volatile uint8_t board_led_pending;
static volatile uint8_t board_led_dirty;

int board_led_init(void)
{
#if defined(PICO_DEFAULT_LED_PIN)
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);
  return PICO_OK;
#elif defined(CYW43_WL_GPIO_LED_PIN)
  // cyw43 は wifi_telemetry_init() で初期化済み
  return PICO_OK;
#else
  return PICO_OK;
#endif
}

static void board_led_apply(uint8_t on)
{
#if defined(PICO_DEFAULT_LED_PIN)
  gpio_put(PICO_DEFAULT_LED_PIN, on);
#elif defined(CYW43_WL_GPIO_LED_PIN)
  cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, on);
#endif
}

void board_led_request(uint8_t on)
{
  board_led_pending = on ? 1 : 0;
  board_led_dirty = 1;
}

void board_led_poll(void)
{
  if (!board_led_dirty)
    return;

  board_led_dirty = 0;
  board_led_apply(board_led_pending);
}
