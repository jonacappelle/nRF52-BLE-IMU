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
 *         File: imu.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Inferfacing with ICM-20948
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdbool.h>

#include "imu_registers.h"
#include "app_fifo.h"
#include "ble_motion_service.h"
#include "usr_util.h"
#include "SensorConfig.h"

//////////////////
// IMU SETTINGS
//////////////////

#define NOMADE_GRYO_FSR             500 // +- 500 dps
#define NOMADE_ACCEL_FSR            4 // 4 G

#define PRINT_MEAS_VALUES           0//1

#define IMU_ENABLED                 1
#define BYPASS_IMU_VDD              1
#define QI_CHG_DETECTION_ENABLED    1

#define COSTUM_BOARD                1
#define SPARKFUN_BOARD              0
#define NRF_DEV_BOARD               0

#if COSTUM_BOARD == 1
#define TIMESYNC_PIN    13
#define IMU_POWER_PIN 14
#define INT_PIN	20
#define QI_CHG_PIN 7
#define USR_TWI_SCL 16
#define USR_TWI_SDA 15
#define USR_RX_PIN_NUMBER   11
#define USR_TX_PIN_NUMBER   12
#endif
#if SPARKFUN_BOARD == 1
#define IMU_POWER_PIN 14
#define TIMESYNC_PIN    17
#define INT_PIN	8
#define USR_TWI_SCL 15
#define USR_TWI_SDA 14

#define USR_RX_PIN_NUMBER   26
#define USR_TX_PIN_NUMBER   27
#endif
#if NRF_DEV_BOARD == 1
#define TIMESYNC_PIN    17
#define IMU_POWER_PIN 14
#define INT_PIN	2
#define QI_CHG_PIN 7
#define USR_TWI_SCL 27
#define USR_TWI_SDA 26
#define USR_RX_PIN_NUMBER  8
#define USR_TX_PIN_NUMBER  6
#endif

#define IMU_DEFAULT_SAMPL_FREQ  4 // 225 Hz

#define RAW_Q_FORMAT_ACC_INTEGER_BITS 6     // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_GYR_INTEGER_BITS 11    // Number of bits used for integer part of raw data.
#define RAW_Q_FORMAT_CMP_INTEGER_BITS 12    // Number of bits used for integer part of raw data.

#define RAW_Q_FORMAT_ACC_COMMA_BITS 10     // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_GYR_COMMA_BITS 5    // Number of bits used for comma part of raw data.
#define RAW_Q_FORMAT_CMP_COMMA_BITS 4    // Number of bits used for comma part of raw data.

#define IMU_FULLY_CALIBRATED_VALUE  3


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

// Structures for keeping track of data

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
    uint8_t gyro_accuracy;
    imu_accel_t accel;
    uint8_t accel_accuracy;
    imu_mag_t mag;
    uint8_t mag_accuracy;
    imu_euler_t euler;
} imu_data_t;

// Create buffer FIFO structure
typedef struct buffer
{
	app_fifo_t quat_fifo;			// Buffer struct for sending QUAT in packets of 10
	uint8_t quat_fifo_buff[1024];	// Buffer allocation for QUAT
	app_fifo_t raw_fifo;			// Buffer struct for sending RAW in packets of 10
	uint8_t raw_fifo_buff[1024];	// Buffer allocation for RAW
} BUFFER;

//////////////////
// IMU Config + Communication
//////////////////

#define ICM_20948_I2C_ADDRESS		0x69U

// Initialization of IMU
void imu_init(void);

// Re-initialization of IMU after sleep mode
void imu_re_init(void);

// IMU de-initialization
void imu_deinit();

// Enable / disable power to IMU
void imu_power_en(bool enable);

// Read IMU calibration values from flash memory and apply to IMU
void apply_stored_offsets(void);

// Get IMU calibration values and store in flash memory
void store_offsets(void);

// Put IMU to sleep and enable WoM
void imu_sleep_wom();

// Enable / disable specific IMU sensors
uint32_t imu_enable_sensors(ble_tms_config_t* self);

// Send the buffered IMU data
void imu_send_data(ble_tms_config_t* p_evt, uint32_t sample_time_ms);

// Set FSR for Gyro and Accel
static void imu_config_fsr_gyro(int fsr_in);
static void imu_config_fsr_accel(int fsr_in);

//////////////////
// BUFFERS
//////////////////

// Init and de-init FIFO buffers
static void imu_buff_init();

// Remove all values from IMU buffers
void imu_clear_buff();

//////////////////
// RAW IMU communication - without Invensense library
// Used to do specific Low-Power things
//////////////////

void ICM20948_reset();
void ICM_20948_wakeOnMotionITEnable(uint8_t womThreshold, float sampleRate);
void ICM_20948_registerWrite(uint16_t addr, uint8_t data);
void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data);
void ICM20948_reset();
uint32_t ICM_20948_sensorEnable(bool accel, bool gyro, bool temp);
uint32_t ICM_20948_lowPowerModeEnter(bool enAccel, bool enGyro, bool enTemp);
uint32_t ICM_20948_interruptEnable(bool dataReadyEnable, bool womEnable);
uint32_t ICM_20948_sleepModeEnable(bool enable);
uint32_t ICM_20948_cycleModeEnable(bool enable);
uint32_t ICM_20948_accelBandwidthSet(uint8_t accelBw);
uint32_t ICM_20948_accelFullscaleSet(uint8_t accelFs);
float ICM_20948_accelSampleRateSet(float sampleRate);
uint32_t ICM_20948_sampleRateSet(float sampleRate);
void ICM_20948_bankSelect(uint8_t bank);

// Check if IMU is in WoM mode
bool imu_in_wom(void);

// Set state of IMU WoM mode
void imu_set_in_wom(bool enable);

// Check if IMU is in shutdown mode
bool imu_in_shutdown(void);

// Set state of IMU shutdown mode
void imu_set_in_shutdown(bool enable);



#endif