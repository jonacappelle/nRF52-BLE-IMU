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

// Struct to keep track of which IMU function are activated
IMU imu = {
	.evt_scheduled = 0,
};

/**@brief Application main function.
 */
int main(void)
{
		ret_code_t err_code;

		// Logging to RTT functionality
    	log_init();

		// Initialize the async SVCI interface to bootloader before any interrupts are enabled.
		dfu_async_init();
		
		// Initialize everything related to BLE
		usr_ble_init();

		// Application scheduler (soft interrupt like)
		app_scheduler_init();
	
		/* Initialize GPIO pins */
		gpio_init();

		/* Init LED */
		led_init();

		// Initialize all needed timers
		imu_timers_init();
	
		// Initialize IMU
		imu_init();

		// Initialize ADC
		usr_adc_init();

		
		//////////////////////////////////////////
		// SPI TODO
		//////////////////////////////////////////
		// Delay before starting
		// nrf_delay_ms(2000);
		// err_code = spi_init();
		// NRF_LOG_INFO("spi err_code; %d", err_code);
		// NRF_LOG_FLUSH();
		// nrf_delay_ms(100);
		// spi_write(0x0A, 0x12);
		// nrf_delay_ms(2000);
		//////////////////////////////////////////
		
		// Main loop	
		while(1)
		{
			// App scheduler: handle event in buffer
			// Execute everything that can't be handled in interrupts - queued operations
			app_sched_execute();	

			// Flush all the debug info to RTT
			NRF_LOG_FLUSH();

			// Enter low power mode when idle
			idle_state_handle();
			// sleep_mode_enter();
			
			// Check for activity of CPU
			// check_cpu_activity();
		}
}
