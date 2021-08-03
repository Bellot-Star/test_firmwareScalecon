#ifndef _MILLIS_H
#define _MILLIS_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "app_timer.h"

/*<Tick : millisecond counter>*/
#define TICK                  10

#define MILLIS_TIMER_TICKS    APP_TIMER_TICKS(TICK)

uint32_t millis(void);
void init_millis_timer(void);
void start_millis_timer();

#endif