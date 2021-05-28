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

// Logging functionality
#define NRF_LOG_MODULE_NAME usr_adc
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "usr_ble.h"


typedef struct{
    float voltage;
} battery_t;

battery_t batt = 
{
    .voltage = 4.2,
};


#define ADC_INPUT_PIN   NRF_SAADC_INPUT_AIN1
#define SAADC_CHANNEL   0

#define R1              4.75    // in MOhm
#define R2              8.2     // in MOhm

#define VOLTAGE_DIVIDER_FACTOR (R1 / (R1 + R2))

#define INVALID_VOLTAGE 0xFF


#define SAMPLES_IN_BUFFER 1
volatile uint8_t state = 1;

static const nrf_drv_timer_t m_timer = NRF_DRV_TIMER_INSTANCE(2);
static nrf_saadc_value_t     m_buffer_pool[2][SAMPLES_IN_BUFFER];
static nrf_ppi_channel_t     m_ppi_channel;


static const nrf_saadc_oversample_t saadc_oversample = NRF_SAADC_OVERSAMPLE_32X;

// TODO
static const nrf_drv_saadc_config_t saadc_config = {                                                                               \
    .resolution         = (nrf_saadc_resolution_t)NRFX_SAADC_CONFIG_RESOLUTION, \
    .oversample         = (nrf_saadc_oversample_t)saadc_oversample, \
    .interrupt_priority = NRFX_SAADC_CONFIG_IRQ_PRIORITY,                       \
    .low_power_mode     = NRFX_SAADC_CONFIG_LP_MODE                             \
};


static const nrf_saadc_channel_config_t channel_config = {
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain       = NRF_SAADC_GAIN1_3, // Gain of 3 -> 0.6V internal REF * 3 = 1.8 V max input
    .reference  = NRF_SAADC_REFERENCE_INTERNAL,
    .acq_time   = NRF_SAADC_ACQTIME_40US, // Large aquisition time, so we can use bigger resistors in the resistor divider
    .mode       = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst      = NRF_SAADC_BURST_ENABLED, // Oversampling for better accuracy during BLE transmission // NRF_SAADC_BURST_DISABLED,
    .pin_p      = (nrf_saadc_input_t)(ADC_INPUT_PIN),
    .pin_n      = NRF_SAADC_INPUT_DISABLED,
};




void timer_handler(nrf_timer_event_t event_type, void * p_context)
{
    NRF_LOG_INFO("Saadc timer handler");
}


void saadc_timer_init()
{
    ret_code_t err_code;

    nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
    timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
    err_code = nrf_drv_timer_init(&m_timer, &timer_cfg, timer_handler);
    APP_ERROR_CHECK(err_code);
}

void saadc_timer_deinit()
{
    // Deinit timer
    nrf_drv_timer_uninit(&m_timer);
}


void saadc_set_timer_evt(uint32_t millis)
{
    /* setup m_timer for compare event every 400ms */
    uint32_t ticks = nrf_drv_timer_ms_to_ticks(&m_timer, millis);
    nrf_drv_timer_extended_compare(&m_timer,
                                   NRF_TIMER_CC_CHANNEL0,
                                   ticks,
                                   NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK,
                                   false);
}

void saadc_timer_start()
{
    nrf_drv_timer_enable(&m_timer);
}

void saadc_timer_stop()
{
    nrf_drv_timer_disable(&m_timer);
}

void saadc_ppi_setup()
{
    ret_code_t err_code;

    uint32_t timer_compare_event_addr = nrf_drv_timer_compare_event_address_get(&m_timer,
                                                                                NRF_TIMER_CC_CHANNEL0);
    uint32_t saadc_sample_task_addr   = nrf_drv_saadc_sample_task_get();

    /* setup ppi channel so that timer compare event is triggering sample task in SAADC */
    err_code = nrf_drv_ppi_channel_alloc(&m_ppi_channel);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_ppi_channel_assign(m_ppi_channel,
                                          timer_compare_event_addr,
                                          saadc_sample_task_addr);
    APP_ERROR_CHECK(err_code);
}


void saadc_sampling_event_init(void)
{
    ret_code_t err_code;

    // Initialize PPI peripheral
    err_code = nrf_drv_ppi_init();
    APP_ERROR_CHECK(err_code);

    // Init timer used to trigger saadc + enable
    saadc_timer_init();

    // Set ADC sample interval -> 10 sec
    saadc_set_timer_evt(10000);

    saadc_timer_start();

    // Setup necessary PPI channels and enable them
    saadc_ppi_setup();
}


void saadc_sampling_event_enable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_enable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

void saadc_sampling_event_disable(void)
{
    ret_code_t err_code = nrf_drv_ppi_channel_disable(m_ppi_channel);

    APP_ERROR_CHECK(err_code);
}

uint8_t usr_adc_voltage_to_percent(float voltage)
{
    if( voltage >= 4.1 ) return 100;
    else if ( voltage >= 4.0 ) return 90;
    else if ( voltage >= 3.9 ) return 70; 
    else if ( voltage >= 3.8 ) return 50;
    else if ( voltage >= 3.7 ) return 30;
    else if ( voltage >= 3.5 ) return 20;
    else if ( voltage >= 3.3 ) return 10;
    else return 0;
}


void saadc_callback(nrf_drv_saadc_evt_t const * p_event)
{
    if (p_event->type == NRF_DRV_SAADC_EVT_DONE)
    {
        ret_code_t err_code;

        err_code = nrf_drv_saadc_buffer_convert(p_event->data.done.p_buffer, SAMPLES_IN_BUFFER);
        APP_ERROR_CHECK(err_code);

        int i;

        //////////////////////////////////////////
        // Resistor divider:
        //          -------     -------
        // 4V2 ----| 8.2 M |---| 4.75 M|---- GND
        //          -------     -------
        //////////////////////////////////////////

        float value;

        for (i = 0; i < SAMPLES_IN_BUFFER; i++)
        {
            value =  p_event->data.done.p_buffer[i] / VOLTAGE_DIVIDER_FACTOR * NRF_SAADC_GAIN1_3 / 1024.0 * 0.6; // 0.6 internal reference

            // Print out ADC Voltage converted values
            NRF_LOG_INFO("ADC -> " NRF_LOG_FLOAT_MARKER "", NRF_LOG_FLOAT(value));
        }

        batt.voltage = value;
        uint8_t percent = usr_adc_voltage_to_percent(batt.voltage);

        // Send out battery level
        batt_level_update(percent);
    }
}


void saadc_init(void)
{
    ret_code_t err_code;

    err_code = nrf_drv_saadc_init(&saadc_config, saadc_callback);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_channel_init(SAADC_CHANNEL, &channel_config);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[0], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_saadc_buffer_convert(m_buffer_pool[1], SAMPLES_IN_BUFFER);
    APP_ERROR_CHECK(err_code);

}

void usr_adc_init()
{
    saadc_init();
    saadc_sampling_event_init();
    saadc_sampling_event_enable();
    NRF_LOG_INFO("SAADC Init");
}

void saadc_deinit()
{
    // Stop the timer
    saadc_timer_stop();

    // Disable timer
    saadc_timer_deinit();

    // Disable PPI channel
    saadc_sampling_event_disable();

    // Free PPI channel
    nrf_drv_ppi_channel_free(m_ppi_channel);

    // Deinit PPI
    nrf_drv_ppi_uninit();

    // Deinit saadc channel
    nrf_drv_saadc_channel_uninit(SAADC_CHANNEL);

    // Deinit saadc pheripheral
    nrf_drv_saadc_uninit();
}

void usr_adc_deinit()
{
    // De-initialize ADC
    // Do this when going to sleep
    saadc_deinit();
}

