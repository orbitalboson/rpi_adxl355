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

void my_callback(ADXL355Fifo * fifo)
{
 // printf("Samples: %d\n", fifo->samples);
 // printf("X: %d\n", fifo->data[0].x);
 // printf("Y: %d\n", fifo->data[0].y);
 // printf("Z: %d\n", fifo->data[0].z);
}

int main(int argc, char * argv[])
{
  printf("Example \n");

  uint8_t t1 = 0b00010000;

  ADXL355_HANDLER spi = {};
  ADXL355Status status = {};
  ADXL355Temperature temp = {};
  spi.spi = 1;
  spi.spi_channel = 0;
  spi.speed = 2000000;

  wiringPiSetup();
  int res = wiringPiSPISetup(spi.spi_channel, spi.speed);

  //adxl355_reset(&spi);
  ADXL355Command cmd;
  uint8_t r;
  ADXL355Fifo fifo = {};
  ADXL355Acceleration acc = {};


  adxl355_measurement_mode(&spi);

  adxl355_fifo_stream(&spi, my_callback, FIFO_STREAM_OVR_BREAK|FIFO_STREAM_FIFO_READ_BREAK);

  adxl355_standby_mode(&spi);

  adxl355_reset(&spi);

  return 0;

  adxl355_get_devid_ad(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_devid_mst(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_partid(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_revid(&spi, &r);
  printf("%02x\n", r);

  adxl355_get_status(&spi, &status);
  adxl355_print_status(&status);


  adxl355_get_power_ctl(&spi, &r);
  printf("adxl355_get_power_ctl %02x\n", r);

  adxl355_measurement_mode(&spi);

  adxl355_get_power_ctl(&spi, &r);
  printf("adxl355_get_power_ctl %02x\n", r);

  adxl355_get_fifo_entries(&spi, &r);
  printf("num %d\n", r);


  adxl355_read_fifo(&spi, &fifo, 3);


  sleep(1);

  adxl355_get_fifo_entries(&spi, &r);
  printf("num %d\n", r);


  adxl355_standby_mode(&spi);

  adxl355_get_power_ctl(&spi, &r);
  printf("adxl355_get_power_ctl %02x\n", r);

  adxl355_read_temperature(&spi, &temp);
  printf("TEMP C: %f \n", temp.celsius);


  adxl355_get_status(&spi, &status);
  adxl355_print_status(&status);

  sleep(1);


  for (int i=0; i<20; i++){
      adxl355_read_acceleration(&spi, &acc);
      printf("X: %d\n", acc.x);
       printf("Y: %d\n", acc.y);
       printf("Z: %d\n", acc.z);
       delay(500);
  }


  adxl355_get_status(&spi, &status);
  adxl355_print_status(&status);

  adxl355_reset(&spi);
  sleep(1);
  adxl355_get_status(&spi, &status);
  adxl355_print_status(&status);


  return 0;
}
