#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdbool.h>

/* Interrupt pin number */

#define TIMESYNC_PIN    17

#define IMU_ENABLED 1

#define COSTUM_BOARD    0
#define NRF_DEV_BOARD   1

#if COSTUM_BOARD == 1
#define INT_PIN	8
#define USR_TWI_SCL 15
#define USR_TWI_SDA 14
#define USR_RX_PIN_NUMBER   26
#define USR_TX_PIN_NUMBER   27
#endif
#if NRF_DEV_BOARD == 1
#define INT_PIN	2
#define USR_TWI_SCL 27
#define USR_TWI_SDA 26
#define USR_RX_PIN_NUMBER  8
#define USR_TX_PIN_NUMBER  6
#endif



#include "usr_util.h"



#define RAW_Q_FORMAT_ACC_INTEGER_BITS 6     // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_GYR_INTEGER_BITS 11    // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_CMP_INTEGER_BITS 12    // Number of bits used for integer part of raw data.

#define RAW_Q_FORMAT_ACC_COMMA_BITS 10     // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_GYR_COMMA_BITS 5    // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_CMP_COMMA_BITS 4    // Number of bits used for comma part of raw data.


/**@brief Motion features.
 */
typedef enum
{
    DRV_MOTION_FEATURE_RAW_ACCEL,
    DRV_MOTION_FEATURE_RAW_GYRO,
    DRV_MOTION_FEATURE_RAW_COMPASS,
    DRV_MOTION_FEATURE_QUAT,
    DRV_MOTION_FEATURE_EULER,
    DRV_MOTION_FEATURE_ROT_MAT,
    DRV_MOTION_FEATURE_HEADING,
    DRV_MOTION_FEATURE_GRAVITY_VECTOR,
    DRV_MOTION_FEATURE_TAP,
    DRV_MOTION_FEATURE_ORIENTATION,
    DRV_MOTION_FEATURE_PEDOMETER,
    DRV_MOTION_FEATURE_WAKE_ON_MOTION
}drv_motion_feature_t;

typedef uint32_t drv_motion_feature_mask_t;

#define DRV_MOTION_FEATURE_MASK_RAW               ((1UL << DRV_MOTION_FEATURE_RAW_ACCEL) | (1UL << DRV_MOTION_FEATURE_RAW_COMPASS) | (1UL << DRV_MOTION_FEATURE_RAW_GYRO))
#define DRV_MOTION_FEATURE_MASK_RAW_ACCEL         (1UL << DRV_MOTION_FEATURE_RAW_ACCEL)
#define DRV_MOTION_FEATURE_MASK_RAW_GYRO          (1UL << DRV_MOTION_FEATURE_RAW_GYRO)
#define DRV_MOTION_FEATURE_MASK_RAW_COMPASS       (1UL << DRV_MOTION_FEATURE_RAW_COMPASS)
#define DRV_MOTION_FEATURE_MASK_QUAT              (1UL << DRV_MOTION_FEATURE_QUAT)
#define DRV_MOTION_FEATURE_MASK_EULER             (1UL << DRV_MOTION_FEATURE_EULER)
#define DRV_MOTION_FEATURE_MASK_ROT_MAT           (1UL << DRV_MOTION_FEATURE_ROT_MAT)
#define DRV_MOTION_FEATURE_MASK_HEADING           (1UL << DRV_MOTION_FEATURE_HEADING)
#define DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR    (1UL << DRV_MOTION_FEATURE_GRAVITY_VECTOR)
#define DRV_MOTION_FEATURE_MASK_TAP               (1UL << DRV_MOTION_FEATURE_TAP)
#define DRV_MOTION_FEATURE_MASK_ORIENTATION       (1UL << DRV_MOTION_FEATURE_ORIENTATION)
#define DRV_MOTION_FEATURE_MASK_PEDOMETER         (1UL << DRV_MOTION_FEATURE_PEDOMETER)
#define DRV_MOTION_FEATURE_MASK_WAKE_ON_MOTION    (1UL << DRV_MOTION_FEATURE_WAKE_ON_MOTION)


typedef struct imu
{
	bool gyro_enabled;
	bool accel_enabled;
	bool mag_enabled;
	bool quat6_enabled;
	bool quat9_enabled;
	bool euler_enabled;
	bool stop;
    bool sync;
    uint64_t sync_start_time;
	uint32_t period; // period in milliseconds (ms)
	uint16_t packet_length;
    bool adc;
}IMU;


typedef struct
{
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
} imu_quat_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_gyro_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_accel_t;

typedef struct
{
    int16_t x;
    int16_t y;
    int16_t z;
} imu_mag_t;

typedef struct
{
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
} imu_euler_t;

typedef struct
{
    imu_quat_t   quat;
    imu_gyro_t gyro;
    imu_accel_t accel;
    imu_mag_t mag;
    imu_euler_t euler;
} imu_data_t;


uint32_t imu_init(void);
void IMU_data_get(uint8_t * data, uint16_t * len);
uint32_t imu_get_bytes_available(void);
void imu_set_bytes_available(uint32_t bytes);
void usr_ringbuf_init(void);
uint32_t imu_enable_sensors(IMU imu);
void set_imu_packet_length(void);
uint32_t IMU_buffer_bytes_available();
void imu_send_data();

#endif
