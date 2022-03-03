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
 *      Created: 2022-03-01
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

// #define USR_LED_SOFTBLINK_DISABLED

static bool qi_chg_enabled = 0;

/* Interrupt pin handeler callback function */
void qi_chg_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{

	NRF_LOG_INFO("Evt handler: QI CHG")
	uint32_t state = 0;

	// Read state
	state = nrf_gpio_pin_read(QI_CHG_PIN);

	if(!state) //  Active low
	{
		// Enable LED
		led_chg_start();

	}else{
		// Disable LED
		led_chg_stop();
	}

}


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

#if QI_CHG_DETECTION_ENABLED == 1

	// CHARGING LED INDICATION
	nrf_drv_gpiote_in_config_t qi_config = GPIOTE_CONFIG_IN_SENSE_TOGGLE(true);

	err_code = nrf_drv_gpiote_in_init(QI_CHG_PIN, &qi_config, qi_chg_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(QI_CHG_PIN, true);

	NRF_LOG_INFO("IMU GPIO Init");
	
#endif

}

static void led_flash()
{
	for(uint8_t c=0; c<4; c++)
	{
		for(uint8_t i=0; i<2; i++)
		{
			led_on();
			nrf_delay_ms(100);
			led_off();
			nrf_delay_ms(100);
		}
		nrf_delay_ms(400);
	}
}

void led_init()
{
	ret_code_t err_code;

	#ifndef USR_LED_SOFTBLINK_DISABLED
	// Set pin mode as output
	nrf_gpio_cfg_output(TIMESYNC_PIN);

	// LED blink on startup
	// while(1)
	// {
		led_flash();
	// }

	// Some delay - may not be necessary
	// nrf_delay_ms(10000);
	#endif
}

void led_deinit()
{
	// Set pin low to disable LED
	led_off();
	// Set to default pin configuration: input with no pull resistors
	// nrf_gpio_cfg_default(TIMESYNC_PIN);
}


void led_on()
{
	#ifndef USR_LED_SOFTBLINK_DISABLED
	if(!qi_chg_enabled) nrf_gpio_pin_set(TIMESYNC_PIN);
	#endif
}

void led_off()
{
	#ifndef USR_LED_SOFTBLINK_DISABLED
	if(!qi_chg_enabled) nrf_gpio_pin_clear(TIMESYNC_PIN);
	#endif
}

void led_toggle()
{
	#ifndef USR_LED_SOFTBLINK_DISABLED
	if(!qi_chg_enabled) nrf_gpio_pin_toggle(TIMESYNC_PIN);
	#endif
}

void led_flash2()
{
	led_on();
	nrf_delay_ms(50);
	led_off();
	nrf_delay_ms(200);
	led_on();
	nrf_delay_ms(50);
	led_off();
}

static void lfclk_init(void)
{
    uint32_t err_code;
    err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}


#define TIMESYNC_PIN_MASK PIN_MASK(TIMESYNC_PIN)

void LED_softblink_start()
{
    ret_code_t err_code;

	// Normally gets initialized by Softdevice
    // lfclk_init();

	// Is already initialized elsewhere before
    // Start APP_TIMER to generate timeouts.
    // err_code = app_timer_init();
    // APP_ERROR_CHECK(err_code);


	led_sb_init_params_t led_sb_init_param = LED_SB_INIT_DEFAULT_PARAMS(TIMESYNC_PIN_MASK);

	led_sb_init_param.active_high = true;
	led_sb_init_param.leds_pin_bm = TIMESYNC_PIN_MASK;
	led_sb_init_param.duty_cycle_min = 5;
	led_sb_init_param.duty_cycle_step = 2;
	led_sb_init_param.duty_cycle_max = 150;
	
	NRF_LOG_INFO("Led Softblink Init");

	err_code = led_softblink_init(&led_sb_init_param);
	APP_ERROR_CHECK(err_code);

	led_softblink_on_time_set(0);
	led_softblink_off_time_set(10000);

	// NRF_LOG_INFO("Led Softblink Start");
	// err_code = led_softblink_start(TIMESYNC_PIN_MASK);
	// APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("Led Softblink Enabled");
}

void led_chg_start()
{
	ret_code_t err_code;

	NRF_LOG_INFO("Led Softblink Start");

	qi_chg_enabled = 1;

	err_code = led_softblink_start(TIMESYNC_PIN_MASK);
	APP_ERROR_CHECK(err_code);
}

void led_chg_stop()
{
	ret_code_t err_code;

	NRF_LOG_INFO("Led Softblink Stop");

	err_code = led_softblink_stop();
	APP_ERROR_CHECK(err_code);

	qi_chg_enabled = 0;
}
