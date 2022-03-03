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
 *         File: usr_adc.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: On-board ADC functionality for reading battery voltage
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef __USR_ADC_H__
#define __USR_ADC_H__

// Includes
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "nrf_drv_saadc.h"
#include "nrf_drv_ppi.h"
#include "nrf_drv_timer.h"
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "app_util_platform.h"
#include "nrf_pwr_mgmt.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


// Initialization and de-init of peripheral
void usr_adc_init();
void usr_adc_deinit();
void saadc_init(void);
void saadc_deinit();
void saadc_sampling_event_enable(void);
void saadc_sampling_event_init(void);


// Successive approximation ADC callback - Data gets pushed here
void saadc_callback(nrf_drv_saadc_evt_t const * p_event);

// Timer for triggering 
void saadc_timer_init();
void saadc_timer_deinit();
void saadc_timer_start();
void saadc_timer_stop();
void saadc_ppi_setup();
void saadc_sampling_event_disable(void);
void saadc_set_timer_evt(uint32_t millis);
// Timer callback
void timer_handler(nrf_timer_event_t event_type, void * p_context);

// Convert voltage readings to percentages
uint8_t usr_adc_voltage_to_percent(float voltage);

#endif
