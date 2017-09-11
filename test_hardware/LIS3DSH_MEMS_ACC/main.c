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
#include "lis3dsh.h"

/****************************************
 * L3GD20H configs with SPI
 ****************************************/

static const SPIConfig spicfg =
{
  NULL,         /* no callback */
  GPIOE,        /* CS line port */
  3,            /* CS line pad number */
  /* CR1 reg */
  SPI_CR1_CPHA | SPI_CR1_CPOL | /* CPHA = 1, CPOL = 1 */
  SPI_CR1_BR_1,                 /* fPCLK prescaler, see reference manual */
  /* CR2 reg */
  0
};

static LIS3DSHDriver lis3dsh;

static const LIS3DSHConfig lis3dshcfg =
{
  LIS3DSH_FS_4G,       /* set range of measurement values to +/- 4G */
  LIS3DSH_ODR_100HZ,   /* set sampling rate of acc to 100Hz */

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
  LIS3DSHDriver *acc_ptr = &lis3dsh;

  /* initialize system */
  halInit();
  chSysInit();

  _start_spi();
  _start_serial();

  /* activate gyroscope */
  lis3dshObjectInit(acc_ptr);          /* initialize LIS3DSH driver */
  lis3dshStart(acc_ptr, &lis3dshcfg);  /* start LIS3DSH driver */
  lis3dshWhoAmI(acc_ptr, &buf);        /* verify SPI communication */

  chprintf((BaseSequentialStream*)&SD4, "WHO_AM_I = %u\r\n", buf);

  while (1) {
    chThdSleepMilliseconds(5);
    if(lis3dshIsDataReady(acc_ptr)) {
      /* read from gyro */
      float acc_readings[LIS3DSH_AXES] = {0.0f};
      lis3dshGetData(acc_ptr, acc_readings);
      chprintf((BaseSequentialStream*)&SD4, "Ax = %0.3f G ; Ay = %0.3f G ; Az = %0.3f G\r\n",
          acc_readings[0], acc_readings[1], acc_readings[2]);
    }
  }

  return 0;
}
