/*
 * reset.c
 *
 *      Author: OrbitalBoson
 */

#include <unistd.h>
#include <stdio.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "adxl355.h"

int main(int argc, char * argv[])
{
  printf("Example \n");

  ADXL355_HANDLER spi = {};
  spi.spi = 1;
  spi.spi_channel = 0;
  spi.speed = 2000000;

  wiringPiSetup();
  int res = wiringPiSPISetup(spi.spi_channel, spi.speed);

  //adxl355_reset(&spi);
  ADXL355Command cmd;
  uint8_t r;
  adxl355_get_devid_ad(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_devid_mst(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_partid(&spi, &r);
  adxl355_print_command_result(&cmd);

  adxl355_get_revid(&spi, &r);
  adxl355_print_command_result(&cmd);

  adxl355_get_status(&spi, &cmd);
  adxl355_print_status(cmd.result.status);

  adxl355_measurement_mode(&spi, &cmd);

  adxl355_get_status(&spi, &cmd);
  adxl355_print_status(cmd.result.status);

  sleep(1);

  adxl355_get_status(&spi, &cmd);
    adxl355_print_status(cmd.result.status);

  adxl355_read_acceleration(&spi, &cmd);

  //while (1){
  adxl355_read_temperature(&spi, &cmd);
  printf("TEMP: %d - %f C\n", cmd.result.temperature.raw, cmd.result.temperature.celsius);
  delay(500);
//  }
  adxl355_read_acceleration(&spi, &cmd);
  printf("X: %d\n", cmd.result.acceleration.x);
  printf("Y: %d\n", cmd.result.acceleration.y);
  printf("Z: %d\n", cmd.result.acceleration.z);
  adxl355_get_status(&spi, &cmd);
     adxl355_print_status(cmd.result.status);

  adxl355_reset(&spi, &cmd);


  return 0;
}
