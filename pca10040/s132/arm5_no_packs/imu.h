#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdbool.h>

/* Interrupt pin number */
#define INT_PIN	2




uint32_t imu_init(void);
void IMU_data_get(uint8_t * send_data);
bool imu_get_bytes_available(void);
void imu_set_bytes_available(bool bytes_available);


#endif
