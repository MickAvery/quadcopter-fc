#include "ch.h"
#include "hal.h"
#include "shell.h"
#include "chprintf.h"
#include "usbcfg.h"

#define GPIO            GPIOB
#define PIN             7U
#define ALT_MODE        2
#define CHANNEL         1
#define PWM_DRV         &PWMD4
#define DUTY_CYCLE_MAX  10000  /* 100% duty cycle */
#define DUTY_CYCLE_MIN  1000   /* 10% duty cycle */

/************************
 * Shell config
 ************************/
#define SHELL_WA_SIZE   THD_WORKING_AREA_SIZE(2048)

static uint32_t DUTY_CYCLE_PERCENT = 5000;

static void max(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  DUTY_CYCLE_PERCENT = DUTY_CYCLE_MAX;
  chprintf(chp, "duty cycle = %u\r\n", DUTY_CYCLE_PERCENT);
  pwmEnableChannel(PWM_DRV, CHANNEL, PWM_PERCENTAGE_TO_WIDTH(PWM_DRV, DUTY_CYCLE_PERCENT));
}

static void min(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  DUTY_CYCLE_PERCENT = DUTY_CYCLE_MIN;
  chprintf(chp, "duty cycle = %u\r\n", DUTY_CYCLE_PERCENT);
  pwmEnableChannel(PWM_DRV, CHANNEL, PWM_PERCENTAGE_TO_WIDTH(PWM_DRV, DUTY_CYCLE_PERCENT));
}

static void increment(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  if( (DUTY_CYCLE_PERCENT += 1000) > DUTY_CYCLE_MAX ) {
    DUTY_CYCLE_PERCENT = DUTY_CYCLE_MAX;
  }
  chprintf(chp, "duty cycle = %u\r\n", DUTY_CYCLE_PERCENT);
  pwmEnableChannel(PWM_DRV, CHANNEL, PWM_PERCENTAGE_TO_WIDTH(PWM_DRV, DUTY_CYCLE_PERCENT));
}

static void decrement(BaseSequentialStream *chp, int argc, char *argv[])
{
  (void)argc;
  (void)argv;
  if( (DUTY_CYCLE_PERCENT -= 1000) < DUTY_CYCLE_MIN ) {
    DUTY_CYCLE_PERCENT = DUTY_CYCLE_MIN;
  }
  chprintf(chp, "duty cycle = %u\r\n", DUTY_CYCLE_PERCENT);
  pwmEnableChannel(PWM_DRV, CHANNEL, PWM_PERCENTAGE_TO_WIDTH(PWM_DRV, DUTY_CYCLE_PERCENT));
}

static const ShellCommand shellcmds[] =
{
  {"max", max},
  {"min", min},
  {"inc", increment},
  {"dec", decrement},
  {NULL, NULL}
};

static const ShellConfig shellcfg =
{
  (BaseSequentialStream*)&SDU1,
  shellcmds
}; 

/************************
 * PWM config
 ************************/
static PWMConfig pwmcfg = {
  40000,
  100,
  NULL,
  {
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL},
    {PWM_OUTPUT_ACTIVE_HIGH, NULL}
  },
  0,
  0
};

int main(void) {
  // thread_t *shellptr;

  /* initialize system */
  halInit();
  chSysInit();

  /* initialize PWM driver, and set pin mode to timer */
  palSetPadMode(GPIO, PIN, PAL_MODE_ALTERNATE(ALT_MODE));
  pwmStart(PWM_DRV, &pwmcfg);

  /* initialize serial-to-usb driver */
  sduObjectInit(&SDU1);
  sduStart(&SDU1, &serusbcfg);

  /* initialize usb driver */
  usbDisconnectBus(serusbcfg.usbp);
  chThdSleepMilliseconds(1500);
  usbStart(serusbcfg.usbp, &usbcfg);
  usbConnectBus(serusbcfg.usbp);

  /* initialize shell and dynamic thread */
  shellInit();

  while (1) {
    if (SDU1.config->usbp->state == USB_ACTIVE) {
      thread_t *shelltp = chThdCreateFromHeap(NULL, SHELL_WA_SIZE,
                                              "shell", NORMALPRIO + 1,
                                              shellThread, (void *)&shellcfg);
      chThdWait(shelltp); /* Waiting termination. */
    }
    chThdSleepMilliseconds(1000);
  }

  return 0;
}
