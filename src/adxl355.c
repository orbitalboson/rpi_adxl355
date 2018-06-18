/*
 * adxl355.c
 *
 *      Author: OrbitalBoson
 */

#include "adxl355.h"

int adxl355_another_spi_write(ADXL355_SPI * spi, uint8_t * data, uint8_t * result, size_t len)
{
  printf("adxl355_another_spi_write");
}

int adxl355_wpi_spi_write(ADXL355_SPI * spi, uint8_t * data, uint8_t * result, size_t len)
{
  int r = wiringPiSPIDataRW(spi->channel, data, len);
  memcpy(result, data, len);

  return r;
}

void
adxl355_print_command_result(ADXL355Command * cmd)
{
  for (uint8_t i=0; i<cmd->len; i++){
    printf("%d - %02x \n", i, cmd->result[i]);
  }
}

void
adxl355_prepare_command(ADXL355Command * cmd){
  cmd->_prepared[0] = cmd->reg;
  for (uint8_t i=1; i<cmd->len; i++){
    cmd->_prepared[i] = cmd->data[i-1];
  }
}

int
adxl355_execute_command(ADXL355_SPI * spi, ADXL355Command * cmd)
{
  uint8_t buf[2];
  buf[0] = cmd->reg;
  buf[1] = cmd->data[0];

  uint8_t result[16];

  _adxl355_spi_write(spi, buf, result, cmd->len);
  printf("%02x\n", buf[0]);
  printf("%02x\n", buf[1]);
}

int adxl355_get_devid_ad(ADXL355_SPI * spi, ADXL355Command * cmd)
{
  cmd->reg = _reg_read(ADXL355_REG_DEVID_AD);
  cmd->len = 2;

  adxl355_prepare_command(cmd);
  _adxl355_spi_write(spi, cmd->_prepared, cmd->result, cmd->len);
}

int adxl355_get_devid_mst(ADXL355_SPI * spi)
{
  ADXL355Command cmd;
  cmd.reg = _reg_read(ADXL355_REG_DEVID_MST);
  cmd.len = 2;

  adxl355_execute_command(spi, &cmd);
}

int adxl355_get_partid(ADXL355_SPI * spi)
{
  ADXL355Command cmd;
  cmd.reg = _reg_read(ADXL355_REG_PARTID);
  cmd.len = 2;

  adxl355_execute_command(spi, &cmd);
}


int adxl355_get_revid(ADXL355_SPI * spi)
{
  ADXL355Command cmd;
  cmd.reg = _reg_read(ADXL355_REG_REVID);
  cmd.len = 2;

  adxl355_execute_command(spi, &cmd);
}

int
adxl355_reset(ADXL355_SPI * spi)
{
  ADXL355Command cmd;
  cmd.reg = _reg_write(ADXL355_REG_RESET);
  cmd.data[0] = ADXL355_RESET_CODE;
  cmd.len = 1;

  adxl355_execute_command(spi, &cmd);
}

