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

void usr_adc_init();
void usr_adc_deinit();

void saadc_init(void);
void saadc_deinit();

void saadc_callback(nrf_drv_saadc_evt_t const * p_event);

void saadc_sampling_event_enable(void);
void saadc_sampling_event_init(void);

void timer_handler(nrf_timer_event_t event_type, void * p_context);

void saadc_timer_init();
void saadc_timer_deinit();

void saadc_set_timer_evt(uint32_t millis);

void saadc_timer_start();
void saadc_timer_stop();

void saadc_ppi_setup();

void saadc_sampling_event_disable(void);

uint8_t usr_adc_voltage_to_percent(float voltage);
