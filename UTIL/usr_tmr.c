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
 *         File: usr_tmr.c
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: Timers functionality
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#include "usr_tmr.h"
#include "usr_gpio.h"
#include "ble_motion_service.h"


/* Timer instance for us timer (TIMER 0)*/
//const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(0);
const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(1);



void imu_timer_init (void)
{
	// TODO: Timer will overflow in 1.19 hours
    ret_code_t err_code;

    const nrfx_timer_config_t timer_config = {
				.frequency = (nrf_timer_frequency_t)NRF_TIMER_FREQ_1MHz,      ///< Frequency 1 MHz
				.mode = (nrf_timer_mode_t)NRF_TIMER_MODE_TIMER,     ///< Mode of operation.
				.bit_width = (nrf_timer_bit_width_t)NRF_TIMER_BIT_WIDTH_32,   ///< Bit width 32 bits
				.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority.
				.p_context = NULL          																	///< Context passed to interrupt handler.
    };

		err_code = nrf_drv_timer_init(&TIMER_MICROS, &timer_config, timer_event_handler);
    	APP_ERROR_CHECK(err_code);
		
		nrf_drv_timer_enable(&TIMER_MICROS);
}

void imu_timer_deinit()
{
	nrf_drv_timer_disable(&TIMER_MICROS);
	nrf_drv_timer_uninit(&TIMER_MICROS);
	NRF_LOG_INFO("IMU timer deinit");
}


/**
 * @brief Handler for timer events.
 */
void timer_event_handler(nrf_timer_event_t event_type, void* p_context)
{
	// TODO: make timer 64 bits and run forever or reset timer when device goes to sleep if measurements are not longer than 1.19 hours
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
            NRF_LOG_INFO("Timer finished");
            break;

        default:
            //Do nothing.
						NRF_LOG_INFO("Timer callback, timer cleared");
						nrf_drv_timer_clear(&TIMER_MICROS); // clear timer when it overflows TODO not tested
            break;
    }
}

void imu_timers_init(void)
{
		/* Initialize us timer */
		imu_timer_init();
}

//////////////////////////////////////////////////
// Timers for dynamically turning on-off TimeSync
//////////////////////////////////////////////////

#include "app_timer.h"
#include "nrf_drv_clock.h"

#include "usr_ble.h"

APP_TIMER_DEF(ts_timer);     /**< Handler for repeated timer used to blink LED 1. */

/**@brief Timeout handler for the repeated timer.
 */
void ts_timer_handler(void * p_context)
{
	// Start timesync again
	TimeSync_re_enable();
}

/**@brief Create timers.
 */
static void create_ts_timer()
{
    ret_code_t err_code;

    // Create single shot timer for enabling TimeSync - TimeSync will be disabled once a valid TimeSync packet gets received
    err_code = app_timer_create(&ts_timer,
                                APP_TIMER_MODE_SINGLE_SHOT,
                                ts_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void ts_timer_init()
{
	// Create TimeSync turn on-off timer
	create_ts_timer();
}

// Start idle timer for x seconds
void ts_start_idle_timer(uint32_t t_sec)
{
	ret_code_t err_code;

	uint32_t t_msec = t_sec * 1000;

	// Start single shot timer
	err_code = app_timer_start(ts_timer, APP_TIMER_TICKS(t_msec), NULL);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("TS timer started");
}

/**@brief Function for initializing the timer module.
 */
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

void ts_timer_stop()
{
	NRF_LOG_INFO("TS timer stopped");
	ret_code_t err_code;
	err_code = app_timer_stop(ts_timer);
	APP_ERROR_CHECK(err_code);
}


//////////////////////////////////////////////////
// Calibration Timers
//////////////////////////////////////////////////

APP_TIMER_DEF(calibration_timer);     /**< Handler for repeated timer used to blink LED */
bool calibration_timer_running = false;

/**@brief Timeout handler for the repeated timer.
 */
static void calibration_timer_handler(void * p_context)
{
    led_toggle();

	// NRF_LOG_INFO("Cal status: g: %d, a: %d, m: %d", imu_data.gyro_accuracy, imu_data.accel_accuracy, imu_data.mag_accuracy);
}

void create_calibration_timer()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&calibration_timer,
                                APP_TIMER_MODE_REPEATED,
                                calibration_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void start_calibration_timer(uint32_t ms)
{
    ret_code_t err_code;

	// Only start when not running already
	if(!calibration_timer_running)
	{
	    err_code = app_timer_start(calibration_timer, APP_TIMER_TICKS(ms), NULL);
    	APP_ERROR_CHECK(err_code);	

		calibration_timer_running = true;
	}	
}

void stop_calibration_timer()
{
    ret_code_t err_code;

	// Only stop timer when timer is running
	if(calibration_timer_running)
	{
		err_code = app_timer_stop(calibration_timer);
    	APP_ERROR_CHECK(err_code);

		calibration_timer_running = false;
	}

    led_off();
}
