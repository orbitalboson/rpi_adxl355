/*
 * adxl355.c
 *
 *      Author: OrbitalBoson
 */

#include "adxl355.h"

int
adxl355_another_spi_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len)
{
  printf("adxl355_another_spi_write");
}

int
adxl355_wpi_spi_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len)
{
  int r = wiringPiSPIDataRW(handler->spi_channel, data, len);
  memcpy(result, data, len);

  return r;
}

int
adxl355_wpi_i2c_write(ADXL355_HANDLER * handler, uint8_t * data, uint8_t * result, size_t len)
{
  printf("adxl355_wpi_i2c_write");
}

void
adxl355_print_command_result(ADXL355Command * cmd)
{
  for (uint8_t i=0; i<cmd->len; i++){
    printf("%d - %02x \n", i, cmd->raw_result[i]);
  }
}

void
adxl355_print_status(uint8_t status)
{
  printf("ADXL355 Status register: \n");
  printf("NVM_BUSY: %s\n", status & ADXL355_STATUS_NVM_BUSY ? "true" : "false");
  printf("ACTIVITY: %s\n", status & ADXL355_STATUS_ACTIVITY ? "true" : "false");
  printf("FIFO_OVR: %s\n", status & ADXL355_STATUS_FIFO_OVR ? "true" : "false");
  printf("FIFO_FULL: %s\n", status & ADXL355_STATUS_FIFO_FULL  ? "true" : "false");
  printf("DATA_RDY: %s\n", status & ADXL355_STATUS_DATA_RDY ? "true" : "false");
}

void
adxl355_prepare_command(ADXL355Command * cmd){
  switch (cmd->reg){
    case ADXL355_REG_READ(ADXL355_REG_DEVID_AD):
    case ADXL355_REG_READ(ADXL355_REG_DEVID_MST):
    case ADXL355_REG_READ(ADXL355_REG_PARTID):
    case ADXL355_REG_READ(ADXL355_REG_REVID):
    case ADXL355_REG_READ(ADXL355_REG_STATUS):
    case ADXL355_REG_READ(ADXL355_REG_FIFO_ENTRIES):
      cmd->len = 2;
      printf("CMD len: %d \n", cmd->len);
      break;
    case ADXL355_REG_READ(ADXL355_REG_TEMP2):
      cmd->len = 3;
      printf("CMD len: %d \n", cmd->len);
      break;
    case ADXL355_REG_READ(ADXL355_REG_XDATA3):
      cmd->len = 10;
      printf("CMD len: %d \n", cmd->len);
      break;
    case ADXL355_REG_READ(ADXL355_REG_FILTER):
    case ADXL355_REG_WRITE(ADXL355_REG_FILTER):
    case ADXL355_REG_READ(ADXL355_REG_FIFO_SAMPLES):
    case ADXL355_REG_WRITE(ADXL355_REG_FIFO_SAMPLES):
    case ADXL355_REG_READ(ADXL355_REG_INT_MAP):
    case ADXL355_REG_WRITE(ADXL355_REG_INT_MAP):
    case ADXL355_REG_READ(ADXL355_REG_SYNC):
    case ADXL355_REG_WRITE(ADXL355_REG_SYNC):
    case ADXL355_REG_READ(ADXL355_REG_RANGE):
    case ADXL355_REG_WRITE(ADXL355_REG_RANGE):
    case ADXL355_REG_READ(ADXL355_REG_POWER_CTL):
    case ADXL355_REG_WRITE(ADXL355_REG_POWER_CTL):
    case ADXL355_REG_WRITE(ADXL355_REG_RESET):
      cmd->len = 2;
      printf("CMD len: %d \n", cmd->len);
      break;
    default:
      printf("Command is unknown!\n");
      break;
  }

  cmd->prepared[0] = cmd->reg;
  for (uint8_t i=1; i<cmd->len; i++){
    cmd->prepared[i] = cmd->data[i-1];
  }

  cmd->status |= ADXL355_COMMAND_PREPARED;
}

int
adxl355_execute_command(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  adxl355_prepare_command(cmd);

  if (handler->spi){
    ADXL355_SPI_WRITE(handler, cmd->prepared, cmd->raw_result, cmd->len);
  }
  else if (handler->i2c) {
    ADXL355_I2C_WRITE(handler, cmd->prepared, cmd->raw_result, cmd->len);
  } else {
      printf("ERROR!!!");
      return ADXL355_ERROR;
  }
}

int adxl355_get_devid_ad(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = _reg_read(ADXL355_REG_DEVID_AD);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int adxl355_get_devid_mst(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = _reg_read(ADXL355_REG_DEVID_MST);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int adxl355_get_partid(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_read(ADXL355_REG_PARTID);

  adxl355_execute_command(handler, cmd);
}


int adxl355_get_revid(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_read(ADXL355_REG_REVID);

  adxl355_execute_command(handler, cmd);
}

int
adxl355_get_status(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_read(ADXL355_REG_STATUS);

  adxl355_execute_command(handler, cmd);
  cmd->result.status = cmd->raw_result[1];
}

int
adxl355_read_temperature(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_read(ADXL355_REG_TEMP2);

  adxl355_execute_command(handler, cmd);

  cmd->result.temperature.raw = ( (cmd->raw_result[1] & 0x0F) << 8) + ( cmd->raw_result[2] );
  cmd->result.temperature.celsius = ((ADXL355_TEMP_NOMINAL_LSB - cmd->result.temperature.raw)
				      / ADXL355_TEMP_NOMINAL_CELSIUS_SLOPE) + ADXL355_TEMP_NOMINAL_CELSIUS;
}

int
adxl355_read_acceleration(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_read(ADXL355_REG_XDATA3);

  adxl355_execute_command(handler, cmd);

  cmd->result.acceleration.x = ( (cmd->raw_result[1] << 24) + (cmd->raw_result[2] << 16) + (cmd->raw_result[3] << 8) >> 12 );
  cmd->result.acceleration.y = ( (cmd->raw_result[4] << 24) + (cmd->raw_result[5] << 16) + (cmd->raw_result[6] << 8) >> 12 );
  cmd->result.acceleration.z = ( (cmd->raw_result[7] << 24) + (cmd->raw_result[8] << 16) + (cmd->raw_result[9] << 8) >> 12 );
}

int
adxl355_measurement_mode(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_write(ADXL355_REG_POWER_CTL);

  cmd->data[0] = 0x00;

  adxl355_execute_command(handler, cmd);
}

int
adxl355_reset(ADXL355_HANDLER * handler, ADXL355Command * cmd)
{
  _clear_command(cmd);
  cmd->reg = _reg_write(ADXL355_REG_RESET);

  cmd->data[0] = ADXL355_RESET_CODE;

  adxl355_execute_command(handler, cmd);
}

