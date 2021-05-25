#include "usr_tmr.h"

#include "ble_tms.h"


/* Timer instance for us timer (TIMER 0)*/
//const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(0);
// temp
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

//		uint32_t time_ticks;
//		time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_MICROS, time_ms);
		
//		nrf_drv_timer_extended_compare(&TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		
		nrf_drv_timer_enable(&TIMER_MICROS);
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



const nrf_drv_timer_t TIMER_DATASEND_1 = NRF_DRV_TIMER_INSTANCE(1);
const nrf_drv_timer_t TIMER_DATASEND_2 = NRF_DRV_TIMER_INSTANCE(2);


void timer_datasend_1_init (void)
{
    ret_code_t err_code;

    const nrfx_timer_config_t timer_config = {
				.frequency = (nrf_timer_frequency_t)NRF_TIMER_FREQ_1MHz,      ///< Frequency 1 MHz
				.mode = (nrf_timer_mode_t)NRF_TIMER_MODE_TIMER,     ///< Mode of operation.
				.bit_width = (nrf_timer_bit_width_t)TIMER_BITMODE_BITMODE_32Bit,   ///< Bit width 32 bits
				.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority.
				.p_context = NULL          																	///< Context passed to interrupt handler.
    };

	err_code = nrf_drv_timer_init(&TIMER_DATASEND_1, &timer_config, timer_datasend_1_event_handler);
	APP_ERROR_CHECK(err_code);

	uint32_t time_ms_100Hz = 10;
	uint32_t time_ticks_100Hz;
	time_ticks_100Hz = nrf_drv_timer_ms_to_ticks(&TIMER_DATASEND_1, time_ms_100Hz);
	
	nrf_drv_timer_extended_compare(&TIMER_DATASEND_1, NRF_TIMER_CC_CHANNEL0, time_ticks_100Hz, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);

	nrf_drv_timer_enable(&TIMER_DATASEND_1);
}

void timer_datasend_2_init (void)
{
    ret_code_t err_code;

    const nrfx_timer_config_t timer_config = {
				.frequency = (nrf_timer_frequency_t)NRF_TIMER_FREQ_1MHz,      ///< Frequency 1 MHz
				.mode = (nrf_timer_mode_t)NRF_TIMER_MODE_TIMER,     ///< Mode of operation.
				.bit_width = (nrf_timer_bit_width_t)TIMER_BITMODE_BITMODE_32Bit,   ///< Bit width 32 bits
				.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority.
				.p_context = NULL          																	///< Context passed to interrupt handler.
    };

	err_code = nrf_drv_timer_init(&TIMER_DATASEND_2, &timer_config, timer_datasend_2_event_handler);
	APP_ERROR_CHECK(err_code);

	uint32_t time_ms_50Hz = 20;
	uint32_t time_ticks_50Hz;
	time_ticks_50Hz = nrf_drv_timer_ms_to_ticks(&TIMER_DATASEND_2, time_ms_50Hz);
	
	nrf_drv_timer_extended_compare(&TIMER_DATASEND_2, NRF_TIMER_CC_CHANNEL0, time_ticks_50Hz, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
	
	nrf_drv_timer_enable(&TIMER_DATASEND_2);
}

extern bool timer_datasend_int;

/**
 * @brief Handler for timer events.
 */
void timer_datasend_1_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
		case NRF_TIMER_EVENT_COMPARE0:
		{
			// NRF_LOG_INFO("Timer 100 Hz");
		}
		break;
				// Pass event to event handler
//				app_sched_event_put(pointer_to_own_data, size_of_data, my_flash_event_func);
				
				// ble_tms_orientation_t adc;	
				// adc.adc_1 = (uint32_t) counterr;

				// uint32_t err_code;
				// err_code = ble_tms_orientation_set(&m_tms, &adc);
				// NRF_LOG_INFO("ble_tms_orientation_set err_code: %d", err_code);						
		break;

        default:
            //Do nothing.
						NRF_LOG_INFO("Timer callback, timer cleared");
						nrf_drv_timer_clear(&TIMER_DATASEND_1);
            break;
    }
}

extern ble_tms_t              m_tms;

/**
 * @brief Handler for timer events.
 */
void timer_datasend_2_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
		case NRF_TIMER_EVENT_COMPARE0:
		{
			// NRF_LOG_INFO("Timer 50 Hz");

			ble_tms_adc_t adc;	
			for(uint8_t i=0; i<40; i++)
			{
				adc.raw[i] = i;
			}

			ret_code_t err_code;
			err_code = ble_tms_adc_set(&m_tms, &adc);
			if(err_code != NRF_SUCCESS)
			{
				NRF_LOG_INFO("ble_tms_orientation_set err_code: %d", err_code);		
			}
		}
		break;
				// Pass event to event handler
//				app_sched_event_put(pointer_to_own_data, size_of_data, my_flash_event_func);
				
				
		break;

        default:
            //Do nothing.
						NRF_LOG_INFO("Timer callback, timer cleared");
						nrf_drv_timer_clear(&TIMER_DATASEND_2);
            break;
    }
}

void imu_timers_init(void)
{
		/* Initialize us timer */
		imu_timer_init();
	
		/* Initialize timer: Generates interrupt at 100 Hz and 50 Hz */
		// timer_datasend_1_init();
		// timer_datasend_2_init();
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
	// Start timer again
	// ts_lp_timer_start();

	// Start timesync again
	TimeSync_re_enable();

    NRF_LOG_INFO("ts_timer_handler");
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

static void ts_lp_timer_start()
{
	ret_code_t err_code;

	// Start single shot timer
	err_code = app_timer_start(ts_timer, APP_TIMER_TICKS(1000), NULL);
	APP_ERROR_CHECK(err_code);
}

void ts_timer_init()
{
	// Create TimeSync turn on-off timer
	create_ts_timer();

	// Start TimeSync turn on-off timer
	// ts_lp_timer_start();
}

// Start idle timer for x seconds
void ts_start_idle_timer(uint32_t t_sec)
{
	ret_code_t err_code;

	uint32_t t_msec = t_sec * 1000;

	// Start single shot timer
	err_code = app_timer_start(ts_timer, APP_TIMER_TICKS(t_msec), NULL);
	APP_ERROR_CHECK(err_code);
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
	app_timer_stop(ts_timer);
}