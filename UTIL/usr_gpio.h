#ifndef _USR_GPIO_H_
#define _USR_GPIO_H_

// GPIO Interrupt
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"




void gpio_init(void);
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

#endif
