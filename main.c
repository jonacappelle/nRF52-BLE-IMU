/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
 *
 * @defgroup ble_sdk_uart_over_ble_main main.c
 * @{
 * @ingroup  ble_sdk_app_nus_eval
 * @brief    UART over BLE application main file.
 *
 * This file contains the source code for a sample application that uses the Nordic UART service.
 * This application uses the @ref srvlib_conn_params module.
 */


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


/**@brief Application main function.
 */
int main(void)
{
		uint32_t err_code;
		
		usr_ble_init();
	
		// Application scheduler (soft interrupt like)
		APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);

	
	// GPIO stuff for timing purposes
	////////////////////////////////	
		nrf_gpio_cfg_output(18);
		nrf_gpio_cfg_output(19);
		nrf_gpio_cfg_output(20);
	////////////////////////////////
		nrf_gpio_cfg_output(25);
		nrf_gpio_pin_set(25);
		nrf_gpio_pin_set(20);
		nrf_delay_ms(1000);
		nrf_gpio_pin_clear(25);
		nrf_gpio_pin_clear(20);
		nrf_delay_ms(1000);
	////////////////////////////////
		

    // Start execution.
    printf("\r\nUART started.\r\n");
    NRF_LOG_INFO("Debug logging for UART over RTT started.");

	
//		APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
//    NRF_LOG_DEFAULT_BACKENDS_INIT();
	
		/* Initialize GPIO pins */
		gpio_init();
		
		/* Initialize us timer */
		timer_init();
		
		/* Initialize timer: Generates interrupt at 100 Hz */
		timer_datasend_init();
		
		// Initialize IMU
		err_code = imu_init();
		APP_ERROR_CHECK(err_code);
		
		
		// Delay before starting
		nrf_delay_ms(5000);
		
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
			if(!nus_buffer_full) 
			{
				uint32_t err_code;
				do
				{
						// Load new data into buffer after NRF_SUCCESS (previous data has successfully been queued)
						if(NUS_send_OK)
						{
//							IMU_data_get();
						}
						// Try to send data over BLE NUS
						err_code = nus_printf_custom("2	Test 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789 0123456789\n\0");
//						err_code = nus_printf_custom("Test 123\n\0");
						
						// Packet has successfully been queued and send correctly
						// If this happens, load new data into buffer
						if(err_code == NRF_SUCCESS)
						{
							NUS_send_OK = true; // Ok, buffer is not full yet, buffer next data
							countrrr++; // Increment send counter
							NRF_LOG_INFO("NUS SUCCESS! %d", countrrr);
						}
						// If NUS send buffer is full, do not load new data into buffer 
						// + stop queue of data until BLE_NUS_EVT_TX_RDY
						if (err_code == NRF_ERROR_RESOURCES)
						{
							NUS_send_OK = false; // NUS send buffer is full
							nus_buffer_full = true; // NUS send buffer is full
//							NRF_LOG_INFO("NUS TX Buffer full!");
						}
				} while (err_code == NRF_SUCCESS);
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




