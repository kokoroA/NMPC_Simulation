// SIL shim: hardware/spi.h（LSM9DS1 は関数レベルで差し替えるので実体不要）
#ifndef _HARDWARE_SPI_H
#define _HARDWARE_SPI_H
#include "pico/types.h"
#ifdef __cplusplus
extern "C" {
#endif
typedef struct spi_inst spi_inst_t;
extern spi_inst_t *spi0;
extern spi_inst_t *spi1;
#define SPI_CPHA_0 0
#define SPI_CPHA_1 1
#define SPI_CPOL_0 0
#define SPI_CPOL_1 1
#define SPI_MSB_FIRST 1
uint spi_init(spi_inst_t *spi, uint baudrate);
void spi_set_format(spi_inst_t *spi, uint data_bits, uint cpol, uint cpha, uint order);
int  spi_write_blocking(spi_inst_t *spi, const uint8_t *src, size_t len);
int  spi_read_blocking(spi_inst_t *spi, uint8_t repeated_tx_data, uint8_t *dst, size_t len);
int  spi_write_read_blocking(spi_inst_t *spi, const uint8_t *src, uint8_t *dst, size_t len);
#ifdef __cplusplus
}
#endif
#endif
