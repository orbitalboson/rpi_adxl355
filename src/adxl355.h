/*
 * adxl355.h
 *
 *      Author: OrbitalBoson
 */

#ifndef ADXL355_H_
#define ADXL355_H_

#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/types.h>

#include <wiringPi.h>
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

#define ADXL355_TEMP_NOMINAL_LSB		1852
#define ADXL355_TEMP_NOMINAL_CELSIUS		25
#define ADXL355_TEMP_NOMINAL_CELSIUS_SLOPE	9.05f
#define ADXL355_TEMP_C_TO_F_CONST		9.0f / 5.0f

#define ADXL355_STATUS_DATA_RDY 	1
#define ADXL355_STATUS_FIFO_FULL 	2
#define ADXL355_STATUS_FIFO_OVR 	4
#define ADXL355_STATUS_ACTIVITY 	8
#define ADXL355_STATUS_NVM_BUSY 	16

#define ADXL355_FIFO_MAX_COUNT		96
#define FIFO_STREAM_OVR_BREAK		0x01
#define FIFO_STREAM_FIFO_READ_BREAK	0x02

#define ADXL355_COMMAND_PREPARED	0x01

#define ADXL355_OK			0
#define ADXL355_ERROR			-1
#define ADXL355_FIFO_READ_ERROR		-10
#define ADXL355_FIFO_STREAM_ERROR	-20

#define ADXL355_REG_WRITE(REG) (REG << 1 | 0x00)
#define ADXL355_REG_READ(REG) (REG << 1 | 0x01)

#define ADXL355_SPI_WRITE(handler, data, result, len) adxl355_wpi_spi_write(handler, data, result, len)
#define ADXL355_I2C_WRITE(handler, data, result, len) adxl355_wpi_i2c_write(handler, data, result, len)

typedef struct {
  unsigned spi : 1;
  unsigned i2c : 1;
  int fd;
  int spi_channel;
  int speed;
} ADXL355_HANDLER;

typedef struct {
  unsigned NVM_BUSY : 1;
  unsigned ACTIVITY : 1;
  unsigned FIFO_OVR : 1;
  unsigned FIFO_FULL : 1;
  unsigned DATA_RDY : 1;
} ADXL355Status;

typedef struct {
  ADXL355Status status;
  uint8_t fifo_entries;
} ADXL355StatusAndFifo;

typedef struct {
  uint16_t raw;
  float celsius;
  float kelvin;
  float fahrenheit;
} ADXL355Temperature;

typedef struct {
  uint32_t x;
  uint32_t y;
  uint32_t z;
} ADXL355Acceleration;

typedef struct {
  ADXL355Acceleration data[ADXL355_FIFO_MAX_COUNT / 3];
  uint8_t samples;
  uint8_t empty_read_index;
  uint8_t x_marker_error_index;
  unsigned empty_read : 1;
  unsigned x_marker_error : 1;
} ADXL355Fifo;

typedef struct {
  uint8_t reg;
  uint8_t status;
  uint8_t data[512];
  uint8_t prepared[512];
  uint8_t raw_result[512];
  size_t len;
} ADXL355Command;

void adxl355_print_command_result(ADXL355Command * cmd);
void adxl355_print_status(ADXL355Status * status);

void adxl355_prepare_command(ADXL355Command * cmd);
int adxl355_execute_command(ADXL355_HANDLER * handler, ADXL355Command * cmd);
int adxl355_wpi_spi_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len);
int adxl355_wpi_i2c_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len);
int adxl355_another_spi_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len);

int adxl355_get_devid_ad(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_get_devid_mst(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_get_partid(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_get_revid(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_get_status(ADXL355_HANDLER * handler, ADXL355Status * status);
int adxl355_get_fifo_entries(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_get_status_n_fifo(ADXL355_HANDLER * handler, ADXL355StatusAndFifo * status);
int adxl355_read_temperature(ADXL355_HANDLER * handler, ADXL355Temperature * temp);
int adxl355_read_acceleration(ADXL355_HANDLER * handler, ADXL355Acceleration * acc);
int adxl355_read_fifo(ADXL355_HANDLER * handler, ADXL355Fifo * fifo, uint8_t entries_count);
int adxl355_fifo_stream(ADXL355_HANDLER * handler, void (* callback)(ADXL355Fifo * fifo), uint8_t flags);
int adxl355_measurement_mode(ADXL355_HANDLER * handler);
int adxl355_standby_mode(ADXL355_HANDLER * handler);
int adxl355_get_power_ctl(ADXL355_HANDLER * handler, uint8_t * b);
int adxl355_reset(ADXL355_HANDLER * handler);

#endif /* ADXL355_H_ */
