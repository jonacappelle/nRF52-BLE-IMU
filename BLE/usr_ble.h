#ifndef _USR_BLE_H_
#define _USR_BLE_H_

#include <stdbool.h>
#include <stdint.h>

uint32_t usr_ble_init(void);
uint32_t nus_printf_custom(char* p_char);
void idle_state_handle(void);


uint32_t nus_send(uint8_t * data, uint16_t len);
void battery_level_update(uint8_t battery_level);
void tms_test(void);

void imu_config_evt_sceduled(void * p_event_data, uint16_t event_size);

#endif
