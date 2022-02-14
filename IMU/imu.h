#ifndef _IMU_H_
#define _IMU_H_

#include <stdint.h>
#include <stdbool.h>

#include "app_fifo.h"

#include "ble_tms.h"

/* Interrupt pin number */

#define IMU_ENABLED                 1
#define BYPASS_IMU_VDD              1
#define QI_CHG_DETECTION_ENABLED    0

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


#include "usr_util.h"



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


// typedef struct imu
// {
// 	bool gyro_enabled;
// 	bool accel_enabled;
// 	bool mag_enabled;
// 	bool quat6_enabled;
// 	bool quat9_enabled;
// 	bool euler_enabled;
// 	bool stop;
//     bool sync;
//     uint64_t sync_start_time;
// 	uint32_t period; // period in milliseconds (ms)
//     bool adc;
//     uint32_t evt_scheduled;
//     bool wom;
//     bool start_calibration;
// }IMU;


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
	app_fifo_t imu_fifo;			// FIFO for IMU data - used by NUS
	uint8_t imu_fifo_buff[1024];		// Buffer for IMU data - used by NUS
	app_fifo_t quat_fifo;			// Buffer struct for sending QUAT in packets of 10
	uint8_t quat_fifo_buff[1024];	// Buffer allocation for QUAT
	app_fifo_t raw_fifo;			// Buffer struct for sending RAW in packets of 10
	uint8_t raw_fifo_buff[1024];	// Buffer allocation for RAW
} BUFFER;

// Flash
void apply_stored_offsets(void);
void store_offsets(void);


void imu_sleep_wom();

void imu_init(void);
void imu_re_init(void);
void imu_deinit();
void imu_power_en(bool enable);

uint32_t imu_enable_sensors(ble_tms_config_t* self);

void imu_send_data(ble_tms_config_t* p_evt, uint32_t sample_time_ms);

// Init and de-init FIFO buffers
static void imu_buff_init();
void imu_clear_buff();

void imu_wom_enable(bool enable);

void imu_set_config();

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

bool imu_in_wom(void);
void imu_set_in_wom(bool enable);
bool imu_in_shutdown(void);
void imu_set_in_shutdown(bool enable);


void create_calibration_timer();
void start_calibration_timer(uint32_t ms);
void stop_calibration_timer();

 #define ICM_20948_WHO_AM_I              0x00                                               
 #define ICM_20948_REG_BANK_SEL  0x7F                        
 #define ICM_20948_BANK_0        ( 0 << 7 )                  
 #define ICM_20948_BANK_1        ( 1 << 7 )                  
 #define ICM_20948_BANK_2        ( 2 << 7 )                  
 #define ICM_20948_BANK_3        ( 3 << 7 )                  
 #define ACCEL_XOUT_H            ( ICM_20948_BANK_0 | 0x2D ) 
 #define ACCEL_XOUT_L            ( ICM_20948_BANK_0 | 0x2E ) 
 #define ACCEL_YOUT_H            ( ICM_20948_BANK_0 | 0x2F ) 
 #define ACCEL_YOUT_L            ( ICM_20948_BANK_0 | 0x30 ) 
 #define ACCEL_ZOUT_H            ( ICM_20948_BANK_0 | 0x31 ) 
 #define ACCEL_ZOUT_L            ( ICM_20948_BANK_0 | 0x32 ) 
 #define GYRO_XOUT_H             ( ICM_20948_BANK_0 | 0x33 ) 
 #define GYRO_XOUT_L             ( ICM_20948_BANK_0 | 0x34 ) 
 #define GYRO_YOUT_H             ( ICM_20948_BANK_0 | 0x35 ) 
 #define GYRO_YOUT_L             ( ICM_20948_BANK_0 | 0x36 ) 
 #define GYRO_ZOUT_H             ( ICM_20948_BANK_0 | 0x37 ) 
 #define GYRO_ZOUT_L             ( ICM_20948_BANK_0 | 0x38 ) 
 #define TEMP_OUT_H              ( ICM_20948_BANK_0 | 0x39 ) 
 #define TEMP_OUT_L              ( ICM_20948_BANK_0 | 0x3A ) 
 /* Gyro Scale */
 #define ICM_20948_SHIFT_GYRO_FS_SEL       1                           
 #define ICM_20948_GYRO_FULLSCALE_250DPS   (0x00 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_500DPS   (0x01 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_1000DPS  (0x02 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_2000DPS  (0x03 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_REG_GYRO_CONFIG_1       (ICM_20948_BANK_2 | 0x01)    
 #define ICM_20948_MASK_GYRO_FULLSCALE     0x06                        
 #define ICM_20948_REG_INT_STATUS          (ICM_20948_BANK_0 | 0x19)    
 #define ICM_20948_BIT_WOM_INT             0x08                        
 #define ICM_20948_REG_INT_ENABLE          (ICM_20948_BANK_0 | 0x10)    
 #define ICM_20948_BIT_WOM_INT_EN          0x08                        
 #define ICM_20948_REG_INT_ENABLE_1        (ICM_20948_BANK_0 | 0x11)    
 #define ICM_20948_BIT_RAW_DATA_0_RDY_EN   0x01                        
 /*******************************************************/
 /***********************/
 /* Bank 0 register map */
 /***********************/
 #define ICM_20948_REG_WHO_AM_I            (ICM_20948_BANK_0 | 0x00)    
 #define ICM_20948_REG_USER_CTRL           (ICM_20948_BANK_0 | 0x03)    
 #define ICM_20948_BIT_DMP_EN              0x80                        
 #define ICM_20948_BIT_FIFO_EN             0x40                        
 #define ICM_20948_BIT_I2C_MST_EN          0x20                        
 #define ICM_20948_BIT_I2C_IF_DIS          0x10                        
 #define ICM_20948_BIT_DMP_RST             0x08                        
 #define ICM_20948_BIT_DIAMOND_DMP_RST     0x04                        
 #define ICM_20948_REG_LP_CONFIG           (ICM_20948_BANK_0 | 0x05)    
 #define ICM_20948_BIT_I2C_MST_CYCLE       0x40                        
 #define ICM_20948_BIT_ACCEL_CYCLE         0x20                        
 #define ICM_20948_BIT_GYRO_CYCLE          0x10                        
 #define ICM_20948_REG_PWR_MGMT_1          (ICM_20948_BANK_0 | 0x06)    
 #define ICM_20948_BIT_H_RESET             0x80                        
 #define ICM_20948_BIT_SLEEP               0x40                        
 #define ICM_20948_BIT_LP_EN               0x20                        
 #define ICM_20948_BIT_TEMP_DIS            0x08                        
 #define ICM_20948_BIT_CLK_PLL             0x01                        
 #define ICM_20948_REG_PWR_MGMT_2          (ICM_20948_BANK_0 | 0x07)    
 #define ICM_20948_BIT_PWR_ACCEL_STBY      0x38                        
 #define ICM_20948_BIT_PWR_GYRO_STBY       0x07                        
 #define ICM_20948_BIT_PWR_ALL_OFF         0x7F                        
 #define ICM_20948_REG_INT_PIN_CFG         (ICM_20948_BANK_0 | 0x0F)    
 #define ICM_20948_BIT_INT_ACTL            0x80                        
 #define ICM_20948_BIT_INT_OPEN            0x40                        
 #define ICM_20948_BIT_INT_LATCH_EN        0x20                        
 #define ICM_20948_REG_INT_ENABLE          (ICM_20948_BANK_0 | 0x10)    
 #define ICM_20948_BIT_WOM_INT_EN          0x08                        
 #define ICM_20948_REG_INT_ENABLE_1        (ICM_20948_BANK_0 | 0x11)    
 #define ICM_20948_BIT_RAW_DATA_0_RDY_EN   0x01                        
 #define ICM_20948_REG_INT_ENABLE_2        (ICM_20948_BANK_0 | 0x12)    
 #define ICM_20948_BIT_FIFO_OVERFLOW_EN_0  0x01                        
 #define ICM_20948_REG_INT_ENABLE_3        (ICM_20948_BANK_0 | 0x13)    
 #define ICM_20948_REG_I2C_MST_STATUS      (ICM_20948_BANK_0 | 0x17)    
 #define ICM_20948_BIT_PASS_THROUGH        0x80
 #define ICM_20948_BIT_SLV4_DONE           0x40
 #define ICM_20948_BIT_LOST_ARB            0x20
 #define ICM_20948_BIT_SLV4_NACK           0x10
 #define ICM_20948_BIT_SLV3_NACK           0x08
 #define ICM_20948_BIT_SLV2_NACK           0x04
 #define ICM_20948_BIT_SLV1_NACK           0x02
 #define ICM_20948_BIT_SLV0_NACK           0x01
  
 #define ICM_20948_REG_INT_STATUS          (ICM_20948_BANK_0 | 0x19)    
 #define ICM_20948_BIT_WOM_INT             0x08                        
 #define ICM_20948_BIT_PLL_RDY             0x04                        
 #define ICM_20948_REG_INT_STATUS_1        (ICM_20948_BANK_0 | 0x1A)    
 #define ICM_20948_BIT_RAW_DATA_0_RDY_INT  0x01                        
 #define ICM_20948_REG_INT_STATUS_2        (ICM_20948_BANK_0 | 0x1B)    
 #define ICM_20948_REG_ACCEL_XOUT_H_SH     (ICM_20948_BANK_0 | 0x2D)    
 #define ICM_20948_REG_ACCEL_XOUT_L_SH     (ICM_20948_BANK_0 | 0x2E)    
 #define ICM_20948_REG_ACCEL_YOUT_H_SH     (ICM_20948_BANK_0 | 0x2F)    
 #define ICM_20948_REG_ACCEL_YOUT_L_SH     (ICM_20948_BANK_0 | 0x30)    
 #define ICM_20948_REG_ACCEL_ZOUT_H_SH     (ICM_20948_BANK_0 | 0x31)    
 #define ICM_20948_REG_ACCEL_ZOUT_L_SH     (ICM_20948_BANK_0 | 0x32)    
 #define ICM_20948_REG_GYRO_XOUT_H_SH      (ICM_20948_BANK_0 | 0x33)    
 #define ICM_20948_REG_GYRO_XOUT_L_SH      (ICM_20948_BANK_0 | 0x34)    
 #define ICM_20948_REG_GYRO_YOUT_H_SH      (ICM_20948_BANK_0 | 0x35)    
 #define ICM_20948_REG_GYRO_YOUT_L_SH      (ICM_20948_BANK_0 | 0x36)    
 #define ICM_20948_REG_GYRO_ZOUT_H_SH      (ICM_20948_BANK_0 | 0x37)    
 #define ICM_20948_REG_GYRO_ZOUT_L_SH      (ICM_20948_BANK_0 | 0x38)    
 #define ICM_20948_REG_TEMPERATURE_H       (ICM_20948_BANK_0 | 0x39)    
 #define ICM_20948_REG_TEMPERATURE_L       (ICM_20948_BANK_0 | 0x3A)    
 #define ICM_20948_REG_TEMP_CONFIG         (ICM_20948_BANK_0 | 0x53)    
 #define ICM_20948_REG_FIFO_EN_1           (ICM_20948_BANK_0 | 0x66)    
 #define ICM_20948_REG_FIFO_EN_2           (ICM_20948_BANK_0 | 0x67)    
 #define ICM_20948_BIT_ACCEL_FIFO_EN       0x10                        
 #define ICM_20948_BITS_GYRO_FIFO_EN       0x0E                        
 #define ICM_20948_REG_FIFO_RST            (ICM_20948_BANK_0 | 0x68)    
 #define ICM_20948_REG_FIFO_MODE           (ICM_20948_BANK_0 | 0x69)    
 #define ICM_20948_REG_FIFO_COUNT_H        (ICM_20948_BANK_0 | 0x70)    
 #define ICM_20948_REG_FIFO_COUNT_L        (ICM_20948_BANK_0 | 0x71)    
 #define ICM_20948_REG_FIFO_R_W            (ICM_20948_BANK_0 | 0x72)    
 #define ICM_20948_REG_DATA_RDY_STATUS     (ICM_20948_BANK_0 | 0x74)    
 #define ICM_20948_BIT_RAW_DATA_0_RDY      0x01                        
 #define ICM_20948_REG_FIFO_CFG            (ICM_20948_BANK_0 | 0x76)    
 #define ICM_20948_BIT_MULTI_FIFO_CFG      0x01                        
 #define ICM_20948_BIT_SINGLE_FIFO_CFG     0x00                        
 /***********************/
 /* Bank 1 register map */
 /***********************/
 #define ICM_20948_REG_XA_OFFSET_H         (ICM_20948_BANK_1 | 0x14)    
 #define ICM_20948_REG_XA_OFFSET_L         (ICM_20948_BANK_1 | 0x15)    
 #define ICM_20948_REG_YA_OFFSET_H         (ICM_20948_BANK_1 | 0x17)    
 #define ICM_20948_REG_YA_OFFSET_L         (ICM_20948_BANK_1 | 0x18)    
 #define ICM_20948_REG_ZA_OFFSET_H         (ICM_20948_BANK_1 | 0x1A)    
 #define ICM_20948_REG_ZA_OFFSET_L         (ICM_20948_BANK_1 | 0x1B)    
 #define ICM_20948_REG_TIMEBASE_CORR_PLL   (ICM_20948_BANK_1 | 0x28)    
 /***********************/
 /* Bank 2 register map */
 /***********************/
 #define ICM_20948_REG_GYRO_SMPLRT_DIV     (ICM_20948_BANK_2 | 0x00)    
 #define ICM_20948_REG_GYRO_CONFIG_1       (ICM_20948_BANK_2 | 0x01)    
 #define ICM_20948_BIT_GYRO_FCHOICE        0x01                        
 #define ICM_20948_SHIFT_GYRO_FS_SEL       1                           
 #define ICM_20948_SHIFT_GYRO_DLPCFG       3                           
 #define ICM_20948_MASK_GYRO_FULLSCALE     0x06                        
 #define ICM_20948_MASK_GYRO_BW            0x39                        
 #define ICM_20948_GYRO_FULLSCALE_250DPS   (0x00 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_500DPS   (0x01 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_1000DPS  (0x02 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_FULLSCALE_2000DPS  (0x03 << ICM_20948_SHIFT_GYRO_FS_SEL)    
 #define ICM_20948_GYRO_BW_12100HZ         (0x00 << ICM_20948_SHIFT_GYRO_DLPCFG)                                     
 #define ICM_20948_GYRO_BW_360HZ           ( (0x07 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_200HZ           ( (0x00 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_150HZ           ( (0x01 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_120HZ           ( (0x02 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_51HZ            ( (0x03 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_24HZ            ( (0x04 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_12HZ            ( (0x05 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_GYRO_BW_6HZ             ( (0x06 << ICM_20948_SHIFT_GYRO_DLPCFG) | ICM_20948_BIT_GYRO_FCHOICE)      
 #define ICM_20948_REG_GYRO_CONFIG_2       (ICM_20948_BANK_2 | 0x02)    
 #define ICM_20948_BIT_GYRO_CTEN           0x38                        
 #define ICM_20948_REG_XG_OFFS_USRH        (ICM_20948_BANK_2 | 0x03)    
 #define ICM_20948_REG_XG_OFFS_USRL        (ICM_20948_BANK_2 | 0x04)    
 #define ICM_20948_REG_YG_OFFS_USRH        (ICM_20948_BANK_2 | 0x05)    
 #define ICM_20948_REG_YG_OFFS_USRL        (ICM_20948_BANK_2 | 0x06)    
 #define ICM_20948_REG_ZG_OFFS_USRH        (ICM_20948_BANK_2 | 0x07)    
 #define ICM_20948_REG_ZG_OFFS_USRL        (ICM_20948_BANK_2 | 0x08)    
 #define ICM_20948_REG_ODR_ALIGN_EN        (ICM_20948_BANK_2 | 0x09)    
 #define ICM_20948_REG_ACCEL_SMPLRT_DIV_1  (ICM_20948_BANK_2 | 0x10)    
 #define ICM_20948_REG_ACCEL_SMPLRT_DIV_2  (ICM_20948_BANK_2 | 0x11)    
 #define ICM_20948_REG_ACCEL_INTEL_CTRL    (ICM_20948_BANK_2 | 0x12)    
 #define ICM_20948_BIT_ACCEL_INTEL_EN      0x02                        
 #define ICM_20948_BIT_ACCEL_INTEL_MODE    0x01                        
 #define ICM_20948_REG_ACCEL_WOM_THR       (ICM_20948_BANK_2 | 0x13)    
 #define ICM_20948_REG_ACCEL_CONFIG        (ICM_20948_BANK_2 | 0x14)    
 #define ICM_20948_BIT_ACCEL_FCHOICE       0x01                        
 #define ICM_20948_SHIFT_ACCEL_FS          1                           
 #define ICM_20948_SHIFT_ACCEL_DLPCFG      3                           
 #define ICM_20948_MASK_ACCEL_FULLSCALE    0x06                        
 #define ICM_20948_MASK_ACCEL_BW           0x39                        
 #define ICM_20948_ACCEL_FULLSCALE_2G      (0x00 << ICM_20948_SHIFT_ACCEL_FS)    
 #define ICM_20948_ACCEL_FULLSCALE_4G      (0x01 << ICM_20948_SHIFT_ACCEL_FS)    
 #define ICM_20948_ACCEL_FULLSCALE_8G      (0x02 << ICM_20948_SHIFT_ACCEL_FS)    
 #define ICM_20948_ACCEL_FULLSCALE_16G     (0x03 << ICM_20948_SHIFT_ACCEL_FS)    
 #define ICM_20948_ACCEL_BW_1210HZ         (0x00 << ICM_20948_SHIFT_ACCEL_DLPCFG)                                    
 #define ICM_20948_ACCEL_BW_470HZ          ( (0x07 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_246HZ          ( (0x00 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_111HZ          ( (0x02 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_50HZ           ( (0x03 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_24HZ           ( (0x04 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_12HZ           ( (0x05 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_ACCEL_BW_6HZ            ( (0x06 << ICM_20948_SHIFT_ACCEL_DLPCFG) | ICM_20948_BIT_ACCEL_FCHOICE)    
 #define ICM_20948_REG_ACCEL_CONFIG_2      (ICM_20948_BANK_2 | 0x15)    
 #define ICM_20948_BIT_ACCEL_CTEN          0x1C                        
 /***********************/
 /* Bank 3 register map */
 /***********************/
 #define ICM_20948_REG_I2C_MST_ODR_CONFIG  (ICM_20948_BANK_3 | 0x00)    
 #define ICM_20948_REG_I2C_MST_CTRL        (ICM_20948_BANK_3 | 0x01)    
 #define ICM_20948_BIT_I2C_MST_P_NSR       0x10                        
 #define ICM_20948_REG_I2C_MST_DELAY_CTRL  (ICM_20948_BANK_3 | 0x02)    
 #define ICM_20948_BIT_SLV0_DLY_EN         0x01                        
 #define ICM_20948_BIT_SLV1_DLY_EN         0x02                        
 #define ICM_20948_BIT_SLV2_DLY_EN         0x04                        
 #define ICM_20948_BIT_SLV3_DLY_EN         0x08                        
 #define ICM_20948_REG_I2C_SLV0_ADDR       (ICM_20948_BANK_3 | 0x03)    
 #define ICM_20948_REG_I2C_SLV0_REG        (ICM_20948_BANK_3 | 0x04)    
 #define ICM_20948_REG_I2C_SLV0_CTRL       (ICM_20948_BANK_3 | 0x05)    
 #define ICM_20948_REG_I2C_SLV0_DO         (ICM_20948_BANK_3 | 0x06)    
 #define ICM_20948_REG_I2C_SLV1_ADDR       (ICM_20948_BANK_3 | 0x07)    
 #define ICM_20948_REG_I2C_SLV1_REG        (ICM_20948_BANK_3 | 0x08)    
 #define ICM_20948_REG_I2C_SLV1_CTRL       (ICM_20948_BANK_3 | 0x09)    
 #define ICM_20948_REG_I2C_SLV1_DO         (ICM_20948_BANK_3 | 0x0A)    
 #define ICM_20948_REG_I2C_SLV2_ADDR       (ICM_20948_BANK_3 | 0x0B)    
 #define ICM_20948_REG_I2C_SLV2_REG        (ICM_20948_BANK_3 | 0x0C)    
 #define ICM_20948_REG_I2C_SLV2_CTRL       (ICM_20948_BANK_3 | 0x0D)    
 #define ICM_20948_REG_I2C_SLV2_DO         (ICM_20948_BANK_3 | 0x0E)    
 #define ICM_20948_REG_I2C_SLV3_ADDR       (ICM_20948_BANK_3 | 0x0F)    
 #define ICM_20948_REG_I2C_SLV3_REG        (ICM_20948_BANK_3 | 0x10)    
 #define ICM_20948_REG_I2C_SLV3_CTRL       (ICM_20948_BANK_3 | 0x11)    
 #define ICM_20948_REG_I2C_SLV3_DO         (ICM_20948_BANK_3 | 0x12)    
 #define ICM_20948_REG_I2C_SLV4_ADDR       (ICM_20948_BANK_3 | 0x13)    
 #define ICM_20948_REG_I2C_SLV4_REG        (ICM_20948_BANK_3 | 0x14)    
 #define ICM_20948_REG_I2C_SLV4_CTRL       (ICM_20948_BANK_3 | 0x15)    
 #define ICM_20948_REG_I2C_SLV4_DO         (ICM_20948_BANK_3 | 0x16)    
 #define ICM_20948_REG_I2C_SLV4_DI         (ICM_20948_BANK_3 | 0x17)    
 #define ICM_20948_BIT_I2C_SLV_EN          0x80                        
 #define ICM_20948_BIT_I2C_BYTE_SW         0x40                        
 #define ICM_20948_BIT_I2C_REG_DIS         0x20                        
 #define ICM_20948_BIT_I2C_GRP             0x10                        
 #define ICM_20948_BIT_I2C_READ            0x80                        
 /* Register common for all banks */
 #define ICM_20948_REG_BANK_SEL            0x7F                        
 // TODO: declaration twice, need fixing
 #define ICM_20948_DEVICE_ID               0xE0                        
 #define ICM20948_DEVICE_ID               0xEA                        
 /********************************************************************/
  
 /* MAGNETOMETER REGISTERS */
  
 #define ICM_20948_I2C_MST_CTRL_CLK_400KHZ    0x07                        
 #define ICM_20948_REG_EXT_SLV_SENS_DATA_00   (ICM_20948_BANK_0 | 0x3B)    
 #define ICM_20948_BIT_I2C_SLV_READ           0x80                        
 /*****************************/
 /* AK09916 register map */
 /*****************************/
 #define AK09916_REG_WHO_AM_I                0x01                        
 #define AK09916_DEVICE_ID                   0x09                        
 #define AK09916_REG_STATUS_1                0x10                        
 #define AK09916_BIT_DRDY                    0x01                        
 #define AK09916_BIT_DOR                     0x02                        
 #define AK09916_REG_HXL                     0x11                        
 #define AK09916_REG_HXH                     0x12                        
 #define AK09916_REG_HYL                     0x13                        
 #define AK09916_REG_HYH                     0x14                        
 #define AK09916_REG_HZL                     0x15                        
 #define AK09916_REG_HZH                     0x16                        
 #define AK09916_REG_STATUS_2                0x18                        
 #define AK09916_REG_CONTROL_2               0x31                        
 #define AK09916_BIT_MODE_POWER_DOWN         0x00                        
 #define AK09916_MODE_SINGLE                 0x01                        
 #define AK09916_MODE_10HZ                   0x02                        
 #define AK09916_MODE_20HZ                   0x04                        
 #define AK09916_MODE_50HZ                   0x06                        
 #define AK09916_MODE_100HZ                  0x08                        
 #define AK09916_MODE_ST                     0x16                        
 #define AK09916_REG_CONTROL_3               0x32                        
 #define AK09916_BIT_SRST                    0x01                        
 #define AK09916_REG_WHO_AM_I                0x01                        
 #define AK09916_BIT_I2C_SLV_ADDR            0x0C    

#endif
