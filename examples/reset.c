/*
 * reset.c
 *
 *      Author: OrbitalBoson
 */

#include <stdio.h>

#include <wiringPi.h>
#include <wiringPiSPI.h>

#include "adxl355.h"

int main(int argc, char * argv[])
{
  printf("Example \n");

  ADXL355_SPI spi;
  spi.channel = 0;
  spi.speed = 2000000;

  wiringPiSetup();
  int res = wiringPiSPISetup(spi.channel, spi.speed);

  //adxl355_reset(&spi);
  ADXL355Command cmd;
  adxl355_get_devid_ad(&spi, &cmd);
  adxl355_print_command_result(&cmd);
  adxl355_get_devid_mst(&spi);
  adxl355_get_partid(&spi);
  adxl355_get_revid(&spi);

  return 0;
}
