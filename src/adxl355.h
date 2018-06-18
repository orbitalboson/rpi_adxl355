/*
 * adxl355.h
 *
 *  Created on: 18 июня 2018 г.
 *      Author: Антон
 */

#ifndef ADXL355_H_
#define ADXL355_H_

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>

#include <wiringPiSPI.h>

#define ADXL355_REG_DEVID_AD		0x00
#define ADXL355_REG_DEVID_MST		0x01
#define ADXL355_REG_PARTID		0x02
#define ADXL355_REG_REVID		0x03
#define ADXL355_REG_STATUS		0x04
#define ADXL355_REG_FIFO_ENTRIES	0x05
#define ADXL355_REG_TEMP2 		0x06
#define ADXL355_REG_TEMP1		0x07
#define ADXL355_REG_XDATA3		0x08
#define ADXL355_REG_XDATA2		0x09
#define ADXL355_REG_XDATA1		0x0A
#define ADXL355_REG_YDATA3		0x0B
#define ADXL355_REG_YDATA2		0x0C
#define ADXL355_REG_YDATA1		0x0D
#define ADXL355_REG_ZDATA3		0x0E
#define ADXL355_REG_ZDATA2		0x0F
#define ADXL355_REG_ZDATA1		0x10
#define ADXL355_REG_FIFO_DATA		0x11
#define ADXL355_REG_OFFSET_X_H 		0x1E
#define ADXL355_REG_OFFSET_X_L 		0x1F
#define ADXL355_REG_OFFSET_Y_H 		0x20
#define ADXL355_REG_OFFSET_Y_L 		0x21
#define ADXL355_REG_OFFSET_Z_H 		0x22
#define ADXL355_REG_OFFSET_Z_L 		0x23
#define ADXL355_REG_ACT_EN		0x24
#define ADXL355_REG_ACT_THRESH_H	0x25
#define ADXL355_REG_ACT_THRESH_L 	0x26
#define ADXL355_REG_ACT_COUNT 		0x27
#define ADXL355_REG_FILTER		0x28
#define ADXL355_REG_FIFO_SAMPLES 	0x29
#define ADXL355_REG_INT_MAP		0x2A
#define ADXL355_REG_SYNC		0x2B
#define ADXL355_REG_RANGE		0x2C
#define ADXL355_REG_POWER_CTL		0x2D
#define ADXL355_REG_RESET		0x2F

#define ADXL355_RESET_CODE		0x52

#define _reg_write(REG) (REG << 1 | 0x00)
#define _reg_read(REG) 	(REG << 1 | 0x01)

#define _adxl355_spi_write(spi, data, result, len) adxl355_wpi_spi_write(spi, data, result, len)

typedef struct {
  int channel;
  int speed;
} ADXL355_SPI;

typedef struct {
  uint8_t reg;
  uint8_t status;
  uint8_t data[16];
  uint8_t _prepared[16];
  uint8_t result[16];
  size_t len;
} ADXL355Command;

void adxl355_prepare_command(ADXL355Command * cmd);
void adxl355_print_command_result(ADXL355Command * cmd);
int adxl355_execute_command(ADXL355_SPI * spi, ADXL355Command * cmd);
int adxl355_wpi_spi_write(ADXL355_SPI * spi, uint8_t * data, uint8_t * result, size_t len);
int adxl355_another_spi_write(ADXL355_SPI * spi, uint8_t * data, uint8_t * result, size_t len);
int adxl355_get_devid_ad(ADXL355_SPI * spi, ADXL355Command * cmd);
int adxl355_get_devid_mst(ADXL355_SPI * spi);
int adxl355_get_partid(ADXL355_SPI * spi);
int adxl355_get_revid(ADXL355_SPI * spi);
int adxl355_reset(ADXL355_SPI * spi);

#endif /* ADXL355_H_ */
