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
 *         File: usr_gpio.c
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: GPIO functionality
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#include "usr_gpio.h"
#include "imu.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "app_gpiote.h"


/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void gpio_init(void)
{
    ret_code_t err_code;

// TODO for some reason, code crashes when this is enabled, this is maybe already enabled elsewhere -> Yes in BSP module
   err_code = nrf_drv_gpiote_init();
   APP_ERROR_CHECK(err_code);




#if IMU_ENABLED == 1
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Low to high trigger

    // Use pulldown for lower power consumption
    // Here we need to use some sort of pull-up / pull-down resistor to detect edges properly
    in_config.pull = NRF_GPIO_PIN_PULLDOWN;

    // TODO: change this in final version
    // Will cause missed trigger events
    in_config.hi_accuracy = false;

    err_code = nrf_drv_gpiote_in_init(INT_PIN, &in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_PIN, true);

	NRF_LOG_INFO("IMU GPIO Init");
#endif
	
	
	// GPIO stuff for timing purposes
	nrf_gpio_cfg_output(PIN_CPU_ACTIVITY);
	nrf_gpio_cfg_output(PIN_IMU_ACTIVITY);
	nrf_gpio_cfg_output(TIMESYNC_PIN); // Timing TS_evt handler

}

static void led_flash()
{
	for(uint8_t c=0; c<2; c++)
	{
		for(uint8_t i=0; i<2; i++)
		{
			nrf_gpio_pin_set(TIMESYNC_PIN);
			nrf_delay_ms(100);
			nrf_gpio_pin_clear(TIMESYNC_PIN);
			nrf_delay_ms(100);
		}
		nrf_delay_ms(400);
	}
}

void led_init()
{
	ret_code_t err_code;

	// Set pin mode as output
	nrf_gpio_cfg_output(TIMESYNC_PIN);

	// LED blink on startup
	led_flash();

	// Some delay - may not be necessary
	// nrf_delay_ms(10000);
}

void led_deinit()
{
	// Set pin low to disable LED
	nrf_gpio_pin_clear(TIMESYNC_PIN);
	// Set to default pin configuration: input with no pull resistors
	// nrf_gpio_cfg_default(TIMESYNC_PIN);
}
