/*  ____  ____      _    __  __  ____ ___
 * |  _ \|  _ \    / \  |  \/  |/ ___/ _ \
 * | | | | |_) |  / _ \ | |\/| | |  | | | |
 * | |_| |  _ <  / ___ \| |  | | |__| |_| |
 * |____/|_| \_\/_/   \_\_|  |_|\____\___/
 *                           research group
 *                             dramco.be/
 *
 *  KU Leuven - Technology Campus Gent,
 *  Gebroeders De Smetstraat 1,
 *  B-9000 Gent, Belgium
 *
 *         File: usr_wdt.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: Watchdog Timer functionality
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

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

// Initialization
void wdt_init();

// Food for the watchdog
void feed_wdt();

// Event handler
void wdt_event_handler(void);

// Check if wake-up source was watchdog
bool is_wdt_wakeup();
void reset_wdt_wakeupt();

#endif
