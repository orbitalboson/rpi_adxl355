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
adxl355_print_status(ADXL355Status * status)
{
  printf("ADXL355 Status register: \n");
  printf("NVM_BUSY: %s\n", status->NVM_BUSY ? "true" : "false");
  printf("ACTIVITY: %s\n", status->ACTIVITY ? "true" : "false");
  printf("FIFO_OVR: %s\n", status->FIFO_OVR ? "true" : "false");
  printf("FIFO_FULL: %s\n", status->FIFO_FULL  ? "true" : "false");
  printf("DATA_RDY: %s\n", status->DATA_RDY ? "true" : "false");
}

void
adxl355_prepare_command(ADXL355Command * cmd){
  if (!cmd->len){
    switch (cmd->reg){
      case ADXL355_REG_READ(ADXL355_REG_DEVID_AD):
      case ADXL355_REG_READ(ADXL355_REG_DEVID_MST):
      case ADXL355_REG_READ(ADXL355_REG_PARTID):
      case ADXL355_REG_READ(ADXL355_REG_REVID):
      case ADXL355_REG_READ(ADXL355_REG_STATUS):
      case ADXL355_REG_READ(ADXL355_REG_FIFO_ENTRIES):
	cmd->len = 2;
	break;
      case ADXL355_REG_READ(ADXL355_REG_TEMP2):
	cmd->len = 3;
	break;
      case ADXL355_REG_READ(ADXL355_REG_XDATA3):
	cmd->len = 10;
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
	break;
      default:
	printf("Command is unknown!\n");
	exit(-1);
	break;
    }
  }

  cmd->prepared[0] = cmd->reg;
  for (uint16_t i=1; i<cmd->len; i++){
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
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_DEVID_AD);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int adxl355_get_devid_mst(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_DEVID_MST);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int adxl355_get_partid(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_PARTID);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}


int adxl355_get_revid(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_REVID);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int
adxl355_get_status(ADXL355_HANDLER * handler, ADXL355Status * status)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_STATUS);

  adxl355_execute_command(handler, &cmd);

  status->NVM_BUSY = cmd.raw_result[1] 	>> 4 & 0x01;
  status->ACTIVITY = cmd.raw_result[1] 	>> 3 & 0x01;
  status->FIFO_OVR = cmd.raw_result[1] 	>> 2 & 0x01;
  status->FIFO_FULL = cmd.raw_result[1]	>> 1 & 0x01;
  status->DATA_RDY = cmd.raw_result[1]	     & 0x01;
}

int
adxl355_get_fifo_entries(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_FIFO_ENTRIES);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1] & 0x7F;
}

int
adxl355_get_status_n_fifo(ADXL355_HANDLER * handler, ADXL355StatusAndFifo * status)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_STATUS);
  cmd.len = 3;

  adxl355_execute_command(handler, &cmd);

  status->status.NVM_BUSY = cmd.raw_result[1] 	>> 4 & 0x01;
  status->status.ACTIVITY = cmd.raw_result[1] 	>> 3 & 0x01;
  status->status.FIFO_OVR = cmd.raw_result[1] 	>> 2 & 0x01;
  status->status.FIFO_FULL = cmd.raw_result[1]	>> 1 & 0x01;
  status->status.DATA_RDY = cmd.raw_result[1]	     & 0x01;

  status->fifo_entries = cmd.raw_result[2] & 0x7F;
}

int
adxl355_read_temperature(ADXL355_HANDLER * handler, ADXL355Temperature * temp)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_TEMP2);

  adxl355_execute_command(handler, &cmd);

  temp->raw = ( (cmd.raw_result[1] & 0x0F) << 8) + ( cmd.raw_result[2] );
  temp->celsius = ((ADXL355_TEMP_NOMINAL_LSB - temp->raw)
		      / ADXL355_TEMP_NOMINAL_CELSIUS_SLOPE) + ADXL355_TEMP_NOMINAL_CELSIUS;
  temp->kelvin = temp->celsius + 273.15;
  temp->fahrenheit = temp->celsius * ADXL355_TEMP_C_TO_F_CONST + 32;
}

int
adxl355_read_acceleration(ADXL355_HANDLER * handler, ADXL355Acceleration * acc)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_XDATA3);

  adxl355_execute_command(handler, &cmd);

  acc->x = ( (cmd.raw_result[1] << 24) + (cmd.raw_result[2] << 16) + (cmd.raw_result[3] << 8) >> 12 );
  acc->y = ( (cmd.raw_result[4] << 24) + (cmd.raw_result[5] << 16) + (cmd.raw_result[6] << 8) >> 12 );
  acc->z = ( (cmd.raw_result[7] << 24) + (cmd.raw_result[8] << 16) + (cmd.raw_result[9] << 8) >> 12 );
}

int
adxl355_read_fifo(ADXL355_HANDLER * handler, ADXL355Fifo * fifo, uint8_t entries_count)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_FIFO_DATA);

  uint8_t remainder;

  if (remainder = entries_count % 3){
    entries_count -= remainder;
  }

  if (entries_count == 0){
    fifo->samples = 0;
    return ADXL355_OK;
  } else if (entries_count > ADXL355_FIFO_MAX_COUNT){
    return ADXL355_ERROR;
  }

  // Each entries_count has 3 bytes of data.
  // Plus one byte for register address.
  cmd.len = 1 + entries_count*3;

  adxl355_execute_command(handler, &cmd);

  uint8_t fifo_samples = entries_count / 3;
  uint8_t x_axis_marker = 0;
  uint8_t empty_indicator = 0;

  for (uint8_t i=0; i<fifo_samples; i++){
    x_axis_marker = cmd.raw_result[1+(i*9)+2] & 0x01;
    empty_indicator = cmd.raw_result[1+(i*9)+2] >> 1 & 0x01;

    if (empty_indicator & !fifo->empty_read) {
      fifo->empty_read = 1;
      fifo->empty_read_index = i;
    }

    if (!x_axis_marker & !fifo->x_marker_error){
      fifo->x_marker_error = 1;
      fifo->x_marker_error_index = i;
    }

    fifo->data[i].x = 		(cmd.raw_result[1+(i*9)] << 24)
			+ 	(cmd.raw_result[1+(i*9)+1] << 16)
			+ 	(cmd.raw_result[1+(i*9)+2] << 8) >> 12;

    fifo->data[i].y = 		(cmd.raw_result[1+(i*9)+3] << 24)
    			+ 	(cmd.raw_result[1+(i*9)+4] << 16)
    			+ 	(cmd.raw_result[1+(i*9)+5] << 8) >> 12;

    fifo->data[i].z = 		(cmd.raw_result[1+(i*9)+6] << 24)
        		+ 	(cmd.raw_result[1+(i*9)+7] << 16)
        		+ 	(cmd.raw_result[1+(i*9)+8] << 8) >> 12;
  }

  fifo->samples = fifo_samples;

  if (fifo->empty_read | fifo->x_marker_error){
    return ADXL355_FIFO_READ_ERROR;
  }

  return ADXL355_OK;
}

int
adxl355_fifo_stream(ADXL355_HANDLER * handler, void (* callback)(ADXL355Fifo * fifo), uint8_t flags)
{
  int res = 0;
  uint8_t fifo_entries = 0;
  uint8_t samples_count = 0;

  ADXL355StatusAndFifo status = {};
  ADXL355Fifo fifo = {};

  while (1){
    adxl355_get_status_n_fifo(handler, &status);

    if (status.status.FIFO_OVR){
      if (flags & FIFO_STREAM_OVR_BREAK){
        printf("FIFO OVERFLOW! \n");
        return ADXL355_FIFO_STREAM_ERROR;
      }
    }

    // Each sample consists of 3 fifo entries: X, Y, Z
    samples_count = status.fifo_entries / 3;

    res = adxl355_read_fifo(handler, &fifo, status.fifo_entries);
    if (res < 0){
      if (flags & FIFO_STREAM_FIFO_READ_BREAK) {
	  printf("x_marker_error %d\n", fifo.x_marker_error);
	  printf("empty_read %d\n", fifo.empty_read);
	  return ADXL355_FIFO_STREAM_ERROR;
      }
    }

    if (fifo.samples > 0){
      callback(&fifo);
    }
/////////////////////////////

/////////////////////////////
  }
}

int
adxl355_measurement_mode(ADXL355_HANDLER * handler)
{
  uint8_t power_ctl = 0;
  adxl355_get_power_ctl(handler, &power_ctl);

  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_WRITE(ADXL355_REG_POWER_CTL);
  cmd.data[0] = power_ctl & 0xFE;

  adxl355_execute_command(handler, &cmd);
}

int
adxl355_standby_mode(ADXL355_HANDLER * handler)
{
  uint8_t power_ctl = 0;
  adxl355_get_power_ctl(handler, &power_ctl);

  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_WRITE(ADXL355_REG_POWER_CTL);
  cmd.data[0] = power_ctl | 0x01;

  adxl355_execute_command(handler, &cmd);
}

int
adxl355_get_power_ctl(ADXL355_HANDLER * handler, uint8_t * b)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_READ(ADXL355_REG_POWER_CTL);

  adxl355_execute_command(handler, &cmd);

  *b = cmd.raw_result[1];
}

int
adxl355_reset(ADXL355_HANDLER * handler)
{
  ADXL355Command cmd = {};
  cmd.reg = ADXL355_REG_WRITE(ADXL355_REG_RESET);
  cmd.data[0] = ADXL355_RESET_CODE;

  adxl355_execute_command(handler, &cmd);
}

