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
 *         File: usr_ble.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Bluetooth Low Energy communication
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_BLE_H_
#define _USR_BLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "ble_motion_service.h"
#include "ble_advertising.h"

// Initialization of BLE connectivity
void usr_ble_init(void);

// Called before going to sleep
// Sets IMU to WoM - closes BLE connection - stops timers - de-inits things
void sleep(void * p_event_data, uint16_t event_size);

// Send battery level
void batt_level_update(uint8_t battery_level);

// Event scheduler for IMU-related activities
void imu_config_evt_sceduled(void * p_event_data, uint16_t event_size);

// Functions to send data over BLE
void ble_send_quat(ble_tms_quat_t * data);
void ble_send_raw(ble_tms_raw_t * data);
void ble_send_adc(ble_tms_adc_t * data);
void ble_send_euler(ble_tms_euler_t * data);

// Configuration stuff
void send_calibration(bool start, bool gyro_done, bool accel_done, bool mag_done);
void sync_info_send(bool state);
bool config_is_calibration_active();
void set_config_calibration(bool state);

// Time synchronization
void sync_timer_init(void);
void TimeSync_re_enable();
void TimeSync_enable();
void ts_set_receiver_started_packet();
void ts_evt_synchronized_disable();


// Disconnect from master - BLE
void ble_disconnect(void);

// Start - stop advertising
void advertising_start(void);
void advertising_stop(void);

// DFU
void dfu_async_init();
void ble_dfu_init();
void peer_manager_init();
void advertising_config_get(ble_adv_modes_config_t * p_config);

#endif
