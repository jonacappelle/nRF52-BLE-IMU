#include "usr_gpio.h"
#include "imu.h"



/**
 * @brief Function for configuring: PIN_IN pin for input, PIN_OUT pin for output,
 * and configures GPIOTE to give an interrupt on pin change.
 */
void gpio_init(void)
{
    ret_code_t err_code;

//    err_code = nrf_drv_gpiote_init();
//    APP_ERROR_CHECK(err_code);


	#ifdef IMU_ENABLED == 1
    nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true); // Low to high trigger
    in_config.pull = NRF_GPIO_PIN_PULLUP;

    err_code = nrf_drv_gpiote_in_init(INT_PIN, &in_config, gpiote_evt_handler);
    APP_ERROR_CHECK(err_code);

    nrf_drv_gpiote_in_event_enable(INT_PIN, true);
	#endif
	
	
		// GPIO stuff for timing purposes
	////////////////////////////////	
		nrf_gpio_cfg_output(18);
		nrf_gpio_cfg_output(19);
		nrf_gpio_cfg_output(20);
		nrf_gpio_cfg_output(17); // Timing TS_evt handler
	////////////////////////////////
		nrf_gpio_cfg_output(25);
		nrf_gpio_pin_set(25);
		nrf_gpio_pin_set(20);
		nrf_delay_ms(1000);
		nrf_gpio_pin_clear(25);
		nrf_gpio_pin_clear(20);
		nrf_delay_ms(1000);
	////////////////////////////////
}
