#ifndef _USR_BLE_H_
#define _USR_BLE_H_

#include <stdbool.h>
#include <stdint.h>

#include "ble_tms.h"

#include "ble_advertising.h"

void usr_ble_init(void);

void usr_ble_nus_init();

void idle_state_handle(void);
void sleep(void * p_event_data, uint16_t event_size);


uint32_t nus_send(uint8_t * data, uint16_t len);
void batt_level_update(uint8_t battery_level);

void imu_config_evt_sceduled(void * p_event_data, uint16_t event_size);

// Functions to send data over BLE
void ble_send_quat(ble_tms_quat_t * data);
void ble_send_raw(ble_tms_raw_t * data);
void ble_send_adc(ble_tms_adc_t * data);
void ble_send_euler(ble_tms_euler_t * data);


void TimeSync_re_enable();
void TimeSync_enable();
void ts_set_receiver_started_packet();

void ble_disconnect(void);

void advertising_start(void);
void advertising_stop(void);

void log_init(void);

void dfu_async_init();

void ble_dfu_init();

void peer_manager_init();

void advertising_config_get(ble_adv_modes_config_t * p_config);

void sync_timer_init(void);

#endif
