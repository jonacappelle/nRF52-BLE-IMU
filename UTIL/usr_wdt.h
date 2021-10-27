#ifndef _USR_WDT_H_
#define _USR_WDT_H_


#include <stdbool.h>
#include <stdint.h>

#include "nrf.h"
#include "bsp.h"
#include "app_timer.h"
#include "app_error.h"
#include "nrf_drv_wdt.h"
#include "nrf_drv_clock.h"
#include "nrf_delay.h"
#include "app_util_platform.h"


void wdt_event_handler(void);
bool is_wdt_wakeup();
void reset_wdt_wakeupt();
void wdt_init();
void feed_wdt();



#endif