#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdbool.h>

/* Interrupt pin number */
#define INT_PIN	2

#include "usr_util.h"

typedef struct imu
{
	bool gyro_enabled;
	bool accel_enabled;
	bool mag_enabled;
	bool quat6_enabled;
	bool quat9_enabled;
	bool euler_enabled;
	bool stop;
	uint32_t period; // period in milliseconds (ms)
	uint16_t packet_length;
}IMU;




uint32_t imu_init(void);
void IMU_data_get(uint8_t * data, uint16_t * len);
uint32_t imu_get_bytes_available(void);
void imu_set_bytes_available(uint32_t bytes);
void usr_ringbuf_init(void);
uint32_t imu_enable_sensors(IMU imu);
void set_imu_packet_length(void);
uint32_t IMU_buffer_bytes_available();

#endif
