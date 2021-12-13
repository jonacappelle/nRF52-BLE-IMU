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
 *         File: main.c
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: BLE Peripheral - IMU + ADC
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#include "main.h"

// Error handling
/**@brief Function for assert macro callback.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyse
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in] line_num    Line number of the failing ASSERT call.
 * @param[in] p_file_name File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}


/**@brief Application main function.
 */
int main(void)
{
		ret_code_t err_code;

		// nrf_delay_ms(1000);

		sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);

		// usr_set_poweroff_mode();

		// idle_state_handle();

		clocks_start();

		// Logging to RTT functionality
    	log_init();

		check_reset_reason();

		// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
		// dfu_async_init();
		
		// Initialize everything related to BL
		usr_ble_init();

		// Application scheduler (soft interrupt like)
		app_scheduler_init();
	
		/* Initialize GPIO pins */
		gpio_init();

		// Soft blink LED when wireless charging or measuring
		LED_softblink_start();

		/* Init LED */
		//led_init();

		// Initialize all needed timers
		imu_timers_init();
	
		// Initialize IMU
		if(is_wdt_wakeup())
		{
			reset_wdt_wakeupt();

			// Power cycle IMU
			imu_power_en(false);
			nrf_delay_ms(500);
		}

		imu_init();	

		// Initialize ADC
		usr_adc_init();

		// Initialize calibration timer
		create_calibration_timer();



		// WDT
		wdt_init();

		
		// Main loop	
		while(1)
		{
			feed_wdt();

			// nrf_delay_ms(6000);

			// App scheduler: handle event in buffer
			// Execute everything that can't be handled in interrupts - queued operations
			app_sched_execute();	

			// Flush all the debug info to RTT
			NRF_LOG_FLUSH();

			feed_wdt();

			// Enter low power mode when idle
			idle_state_handle();
			// sleep_mode_enter();
			
			// Check for activity of CPU
			// check_cpu_activity();
		}
}
