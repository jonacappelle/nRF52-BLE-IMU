// nRF52 BLE IMU
// Jona Cappelle


#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

///////////////////////////////////////////////

#include "nrf_delay.h"


// IMU
////////////////
//  INCLUDES  //
////////////////
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "Invn/Devices/SerifHal.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"
#include "Invn/EmbUtils/Message.h"

// Own includes
#include "imu.h"
#include "string.h"
#include "usr_twi.h"
#include "usr_tmr.h"
#include "usr_gpio.h"

// TimeSync
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

// Application scheduler
#include "app_scheduler.h"


// User bluetooth BLE
#include "usr_ble.h"

// Utilities user
#include "usr_util.h"

// Ringbuffer
#include "nrf_ringbuf.h"

// IMU params
#include "imu_params.h"


////////////////
//  DEFINES   //
////////////////

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

/*
 * Last time at which 20948 IRQ was fired
 */
static volatile uint32_t last_irq_time = 0;



uint64_t inv_icm20948_get_dataready_interrupt_time_us(void)
{
	return last_irq_time;
}


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


// Temp external imports of datasend
extern bool NUS_send_OK;
extern bool nus_buffer_full;
extern int countrrr;


/// Timer at 100 Hz
bool timer_datasend_int = false;


// Event handler for scheduler
void my_app_sched_event_handler(void *data, uint16_t size);

// IMU data
float send_data[4];

// Struct to keep track of which IMU function are activated
IMU imu;


/**@brief Application main function.
 */
int main(void)
{
		uint32_t err_code;
		
		usr_ble_init();
	
		// Application scheduler (soft interrupt like)
		APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
		
    // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");
	
		/* Initialize GPIO pins */
		gpio_init();
		
		// Initialize all needed timers
		timers_init();
		
		// IMU Settings
//		imu.quat9_enabled = true;
		imu.period = 5; // 225 Hz
	
		// Initialize IMU
		err_code = imu_init();
		APP_ERROR_CHECK(err_code);
		
		// Initialize ringbuffer
		usr_ringbuf_init();
		
		// Delay before starting
		nrf_delay_ms(2000);
		
		////////////////////////////////////////////////////////////////	
		// Loop: IMU gives interrupt -> bool interrupt = true -> poll device for data
		////////////////////////////////////////////////////////////////		
		while(1)
		{
		
			// App scheduler: handle event in buffer
			app_sched_execute();
			
/////////////////////////////////////////////////////////////////////////////////	
//// Send data over BLE as fast as possible			
/////////////////////////////////////////////////////////////////////////////////
			
			// if NUS TX buffer isn't full and imu_bytes_available() TODO add
			if((!nus_buffer_full) && (imu_get_bytes_available() > 0))
			{
				uint32_t err_code;
				do
				{
						// Stop sending data when FIFO buffer is empty
						if(imu_get_bytes_available() == 0)
						{
							break;
						}
						// Load new data into buffer after NRF_SUCCESS (previous data has successfully been queued)
						if(NUS_send_OK)
						{
							IMU_data_get(send_data);
							// Decrement available bytes once a byte has been send
							uint32_t bytes_available = imu_get_bytes_available();
							bytes_available--;
							imu_set_bytes_available(bytes_available);
							NRF_LOG_INFO("B: %d", bytes_available);
//							NRF_LOG_INFO("%d %d %d %d", (int)(send_data[0]*1000), (int)(send_data[1]*1000), (int)(send_data[2]*1000), (int)(send_data[3]*1000));
						}
						// Try to send data over BLE NUS
//						err_code = nus_printf_custom("2	Test 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789\n\0");
						err_code = nus_send( (uint8_t *) send_data, sizeof(send_data) );
						
						// Packet has successfully been queued and send correctly
						// If this happens, load new data into buffer
						if(err_code == NRF_SUCCESS)
						{
							NUS_send_OK = true; // Ok, buffer is not full yet, buffer next data
							countrrr++; // Increment send counter
//							NRF_LOG_INFO("NUS SUCCESS! %d", countrrr);
						}
						// If NUS send buffer is full, do not load new data into buffer 
						// + stop queue of data until BLE_NUS_EVT_TX_RDY
						if (err_code == NRF_ERROR_RESOURCES)
						{
							NUS_send_OK = false; // NUS send buffer is full
							nus_buffer_full = true; // NUS send buffer is full
//							NRF_LOG_INFO("NUS TX Buffer full!");
						}
				} while ((err_code == NRF_SUCCESS));
			}
			

			
/////////////////////////////////////////////////////////////////////////////////
			
			// Flush all the debug info to RTT
			NRF_LOG_FLUSH();
			
			// Check for activity of CPU
			nrf_gpio_pin_clear(18);
			
			/* Enter low power mode when idle */
			idle_state_handle();
			
			// Check for activity of CPU
			nrf_gpio_pin_set(18);
			
		}// while(1)
}// main




