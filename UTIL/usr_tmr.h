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

void timers_init(void);
void imu_timers_init(void);


void timer_event_handler(nrf_timer_event_t event_type, void* p_context);

void timer_datasend_1_init (void);
void timer_datasend_2_init (void);
void timer_datasend_1_event_handler(nrf_timer_event_t event_type, void* p_context);
void timer_datasend_2_event_handler(nrf_timer_event_t event_type, void* p_context);

void ts_timer_handler(void * p_context);
static void create_ts_timer();
static void ts_lp_timer_start();
void ts_timer_init();

void ts_start_idle_timer(uint32_t t_sec);

void ts_timer_stop();


#endif
