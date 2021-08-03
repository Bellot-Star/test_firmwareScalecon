#include "millis.h"

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_error.h"
#include "app_util.h"

static uint32_t time_elapsed = 0;
APP_TIMER_DEF(timer_millis);

uint32_t millis(void){
  return time_elapsed;
}

static void millis_timeout_handler(void *p_context){
  time_elapsed++;
}

void init_millis_timer(void){
  ret_code_t err_code;
  err_code = app_timer_create(&timer_millis, APP_TIMER_MODE_REPEATED, millis_timeout_handler);
  APP_ERROR_CHECK(err_code);
}

void start_millis_timer()
{
  ret_code_t err_code;
  err_code = app_timer_start(timer_millis, MILLIS_TIMER_TICKS, NULL);
  APP_ERROR_CHECK(err_code);
}

