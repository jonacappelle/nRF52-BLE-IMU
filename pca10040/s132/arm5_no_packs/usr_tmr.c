#include "usr_tmr.h"


/* Timer instance for us timer (TIMER 0)*/
//const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(0);
// temp
const nrf_drv_timer_t TIMER_MICROS = NRF_DRV_TIMER_INSTANCE(4);



void timer_init (void)
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



const nrf_drv_timer_t TIMER_DATASEND = NRF_DRV_TIMER_INSTANCE(1);


void timer_datasend_init (void)
{
    ret_code_t err_code;

    const nrfx_timer_config_t timer_config = {
				.frequency = (nrf_timer_frequency_t)NRF_TIMER_FREQ_1MHz,      ///< Frequency 1 MHz
				.mode = (nrf_timer_mode_t)NRF_TIMER_MODE_TIMER,     ///< Mode of operation.
				.bit_width = (nrf_timer_bit_width_t)TIMER_BITMODE_BITMODE_32Bit,   ///< Bit width 32 bits
				.interrupt_priority = NRFX_TIMER_DEFAULT_CONFIG_IRQ_PRIORITY, ///< Interrupt priority.
				.p_context = NULL          																	///< Context passed to interrupt handler.
    };

		err_code = nrf_drv_timer_init(&TIMER_DATASEND, &timer_config, timer_datasend_event_handler);
    APP_ERROR_CHECK(err_code);

		uint32_t time_ms = 10; // 1s - needs to change to 10 ms for 100 Hz
		uint32_t time_ticks;
		time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_DATASEND, time_ms);
		
		nrf_drv_timer_extended_compare(&TIMER_DATASEND, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
		
		nrf_drv_timer_enable(&TIMER_DATASEND);
}

extern bool timer_datasend_int;

/**
 * @brief Handler for timer events.
 */
void timer_datasend_event_handler(nrf_timer_event_t event_type, void* p_context)
{
    switch (event_type)
    {
        case NRF_TIMER_EVENT_COMPARE0:
//				NRF_LOG_INFO("Timer interrupt!");
				nrf_gpio_pin_set(19);
				
				timer_datasend_int = true;
						
				nrf_gpio_pin_clear(19);
            break;

        default:
            //Do nothing.
						NRF_LOG_INFO("Timer callback, timer cleared");
						nrf_drv_timer_clear(&TIMER_DATASEND);
            break;
    }
}
