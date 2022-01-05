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
 *         File: main.h
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: BLE Peripheral - IMU + ADC
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#ifndef _MAIN_H_
#define _MAIN_H_


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

#include "../Devices/SerifHal.h"
#include "../Devices/DeviceIcm20948.h"
#include "../DynamicProtocol/DynProtocol.h"
#include "../DynamicProtocol/DynProtocolTransportUart.h"
#include "../EmbUtils/Message.h"

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

// TMS Service
#include "ble_tms.h"

// BLE Battery service
#include "ble_bas.h"

// EMG
#include "emg.h"

// BLE NUS Functionality
#include "usr_ble_nus.h"


// Flash
#include "usr_flash.h"

// ADC
#include "usr_adc.h"

// Watchdog timer
#include "usr_wdt.h"



////////////////
//  DEFINES   //
////////////////

#define DEAD_BEEF   0xDEADBEEF          /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */



#endif