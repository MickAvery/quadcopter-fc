/**
 * \file   main.c
 * \author Mav Cuyugan
 * \brief  App entry point for L3GD20H hardware test
 */

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "ch.h"
#include "hal.h"
#include "chprintf.h"
#include "l3gd20h.h"

#define MAX_STRING_LEN                256U

/****************************************
 * L3GD20H configs with SPI
 ****************************************/

static const SPIConfig spicfg =
{
  NULL,         /* no callback */
  GPIOA,        /* CS line port */
  15,           /* CS line pad number */
  /* CR1 reg */
  SPI_CR1_CPHA | SPI_CR1_CPOL | /* CPHA = 1, CPOL = 1 */
  SPI_CR1_BR_1,                 /* fPCLK prescaler, see reference manual */
  /* CR2 reg */
  0
};

static L3GD20HDriver l3gd20h;

static const L3GD20HConfig l3gd20hcfg =
{
  L3GD20H_FULLSCALE_500DPS, /* set range of measurement values to +/- 500dps */
  L3GD20H_ODR_100Hz,        /* set sampling rate of gyro to 200Hz */

  /* SPI driver pointer */
  &SPID1
};

/****************************************
 * Private functions
 ****************************************/

/**
 * \brief Start SPI peripheral
 */
static void
_start_spi(void)
{
  /* SCK */
  palSetPadMode(GPIOA, 5, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  /* MISO */
  palSetPadMode(GPIOA, 6, PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  /* MOSI */
  palSetPadMode(GPIOA, 7,
                PAL_MODE_ALTERNATE(5) | PAL_STM32_OSPEED_HIGHEST);
  /* CS */
  palSetPadMode(GPIOA, 15, PAL_MODE_OUTPUT_PUSHPULL | PAL_STM32_OSPEED_HIGHEST);
  /* set CS high */
  palSetPad(GPIOA, 15);
  spiStart(&SPID1, &spicfg);
}

/**
 * \brief Start Serial peripheral
 */
static void
_start_serial(void)
{
  /* UART_TX */
  palSetPadMode(GPIOA, 0, PAL_MODE_ALTERNATE(8));
  /* UART_RX */
  palSetPadMode(GPIOA, 1, PAL_MODE_ALTERNATE(8));
  sdStart(&SD4, NULL);
}

/****************************************
 * main function
 ****************************************/

int main(void) {
  uint8_t buf;
  L3GD20HDriver *gyro_ptr = &l3gd20h;

  /* initialize system */
  halInit();
  chSysInit();

  _start_spi();
  _start_serial();

  /* activate gyroscope */
  l3gd20hObjectInit(gyro_ptr);         /* initialize L3GD20H driver */
  l3gd20hStart(gyro_ptr, &l3gd20hcfg); /* start L3GD20H driver */
  l3gd20hWhoAmI(gyro_ptr, &buf);       /* verify SPI communication */
  l3gd20hCalibrate(gyro_ptr);          /* calibrate for bias */

  chprintf((BaseSequentialStream*)&SD4, "WHO_AM_I = %u\r\n", buf);

  while (1) {
    chThdSleepMilliseconds(5);
    if(l3gd20hIsDataReady(gyro_ptr)) {
      /* read from gyro */
      float gyro_readings[L3GD20H_AXES] = {0};
      l3gd20hReadCooked(gyro_ptr, gyro_readings);
      chprintf((BaseSequentialStream*) &SD4, "Gx = %0.3f dps ; Gy = %0.3f dps ; Gz = %0.3f dps\r\n",
          gyro_readings[0], gyro_readings[1], gyro_readings[0]);
    }
  }

  return 0;
}
