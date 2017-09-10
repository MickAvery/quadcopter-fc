/**
 * \file   main.c
 * \author Mav Cuyugan
 * \brief  Main app point of entry
 **/

#include "ch.h"
#include "hal.h"

int main(void) {

  /* initialize system */
  halInit();
  chSysInit();

  while (1) {
    chThdSleepMilliseconds(1000);
  }

  return 0;
}
