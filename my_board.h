#ifndef _MY_BOARD_H
#define _MY_BOARD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_gpio.h"

/**< pinmap : refer to board pcb >*/

//#define STATUS_LED              NRF_GPIO_PIN_MAP(0, 14)
//#define ADDR0                   NRF_GPIO_PIN_MAP(1, 13)
//#define ADDR1                   NRF_GPIO_PIN_MAP(1, 15)
//#define ADDR2                   NRF_GPIO_PIN_MAP(0, 2)
//#define ADDR3                   NRF_GPIO_PIN_MAP(1, 9)
//#define ADDR4                   NRF_GPIO_PIN_MAP(0, 6)
//#define ADDR5                   NRF_GPIO_PIN_MAP(0, 8)

//#define ENABLE1                 NRF_GPIO_PIN_MAP(1, 10)
//#define ENABLE2                 NRF_GPIO_PIN_MAP(0, 26)
#define FORCE_ON                NRF_GPIO_PIN_MAP(1, 13)
#define FORCE_OFF               NRF_GPIO_PIN_MAP(1, 15)

#define UART_RX_PIN    	      	NRF_GPIO_PIN_MAP(0, 6)
#define UART_TX_PIN     	NRF_GPIO_PIN_MAP(0, 8)

#define STATUS_LED              NRF_GPIO_PIN_MAP(0, 13)
#endif