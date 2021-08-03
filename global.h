#ifndef _GLOBAL_H
#define _GLOBAL_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "millis.h"
/**< Name of device. Will be included in the advertising data.> */
#define DEVICE_NAME     "Scale_v1"

#define CHANNELS        20

/**< do not enable uart. This will make current upto 1.2mA > */
#define UART_ENABLE

//#define ENABLE_DEBUG_LOG

enum APP_MODE{
  APP_MODE_INIT,    /* initializing */
  APP_MODE_10HZ,    /* running */
  APP_MODE_1HZ,     /* walking */
  APP_MODE_SLEEP    /* sitting or sleeping */
};

#define TIMEOVER_10HZ       100/TICK    //  100ms
#define TIMEOVER_1HZ        1000/TICK   //  1s
#define TIMEOVER_SLEEP      10000/TICK  //  10s
#define TIMEOVER_GET_BATT   5000/TICK   //  5s

#define DEFULAT_MODE      APP_MODE_1HZ
/**< running mode. Will make the device running in APP_MODE.> */
extern uint8_t g_app_mode;

#endif