// SIL shim: hardware/i2c.h
#ifndef _HARDWARE_I2C_H
#define _HARDWARE_I2C_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct i2c_inst i2c_inst_t;
extern i2c_inst_t *i2c0;
extern i2c_inst_t *i2c1;
#define i2c_default i2c0
uint i2c_init(i2c_inst_t *i2c, uint baudrate);
int  i2c_write_blocking(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop);
int  i2c_read_blocking(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop);
int  i2c_write_timeout_us(i2c_inst_t *i2c, uint8_t addr, const uint8_t *src, size_t len, bool nostop, uint timeout_us);
int  i2c_read_timeout_us(i2c_inst_t *i2c, uint8_t addr, uint8_t *dst, size_t len, bool nostop, uint timeout_us);
#ifdef __cplusplus
}
#endif
#endif
