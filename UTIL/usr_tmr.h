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
 *         File: usr_tmr.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: Timers functionality
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#ifndef _USR_TMR_H_
#define _USR_TMR_H_

// Timer
#include <stdbool.h>
#include <stdint.h>
#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


// Initialization
void timers_init(void);
void imu_timers_init(void);
void imu_timer_deinit();

// TimeSync turn on-off timers
void ts_timer_init();
static void create_ts_timer();
void ts_start_idle_timer(uint32_t t_sec);
void ts_timer_stop();

// IMU timer event handler
void timer_event_handler(nrf_timer_event_t event_type, void* p_context);
// TimeSync timer event handler
void ts_timer_handler(void * p_context);

//////////////////
// Calibration Timers
//////////////////

// Initialize calibration timer
void create_calibration_timer();

// Start calibration timer
void start_calibration_timer(uint32_t ms);

// Stop calibration timer
void stop_calibration_timer();

#endif
