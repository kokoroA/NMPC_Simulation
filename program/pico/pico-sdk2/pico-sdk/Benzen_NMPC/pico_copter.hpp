#ifndef PICO_COPTER_HPP
#define PICO_COPTER_HPP

#include <string.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/binary_info.h"
#include "sensor.hpp"
#include "ekf.hpp"
#include "pwm.hpp"
#include "radio.hpp"
#include "control.hpp"
#include "i2c.hpp"
#include "rgbled.hpp"
#include "board_led.hpp"
#include <math.h>

#define MAINLOOP loop_400Hz

//グローバル変数
extern uint8_t Arm_flag;
extern uint8_t Red_flag;

extern semaphore_t sem;

#endif

