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

// Softblink LED
#include "led_softblink.h"
#include "app_timer.h"
#include "nrf_drv_clock.h"
#include "app_util_platform.h"


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
    // in_config.hi_accuracy = false;

    err_code = nrf_drv_gpiote_in_init(INT_PIN, &in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_PIN, true);

	NRF_LOG_INFO("IMU GPIO Init");
#endif
	
	
	// GPIO stuff for timing purposes
	// nrf_gpio_cfg_output(PIN_CPU_ACTIVITY);
	// nrf_gpio_cfg_output(PIN_IMU_ACTIVITY);

}

static void led_flash()
{
	for(uint8_t c=0; c<4; c++)
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
	// while(1)
	// {
		led_flash();
	// }
	

	// while(1){
	// 	nrf_gpio_pin_set(TIMESYNC_PIN);
	// 	nrf_delay_ms(1000);
	// 	nrf_gpio_pin_clear(TIMESYNC_PIN);
	// 	nrf_delay_ms(1000);
	// }


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


void LED_softblink_start()
{
    ret_code_t err_code;

	// Normally gets initialized by Softdevice
    // lfclk_init();

	// Is already initialized elsewhere before
    // // Start APP_TIMER to generate timeouts.
    // err_code = app_timer_init();
    // APP_ERROR_CHECK(err_code);



	led_sb_init_params_t led_sb_init_param = LED_SB_INIT_DEFAULT_PARAMS(13);

	led_sb_init_param.active_high = true;

	// {                                                                   \
	// 	.active_high        = LED_SB_INIT_PARAMS_ACTIVE_HIGH,           \
	// 	.duty_cycle_max     = 220,        \
	// 	.duty_cycle_min     = 0,        \
	// 	.duty_cycle_step    = 5,       \
	// 	.off_time_ticks     = 65536,        \
	// 	.on_time_ticks      = 0,         \
	// 	.leds_pin_bm        = TIMESYNC_PIN,     \
	// 	.p_leds_port        = NRF_GPIO              \
	// };
	
	NRF_LOG_INFO("Led Softblink Init");

	err_code = led_softblink_init(&led_sb_init_param);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("Led Softblink Start");

	err_code = led_softblink_start(13);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("Led Softblink Enabled");
}
