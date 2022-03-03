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
 *         File: imu_registers.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Register description for ICM-20948
 *
 *  Commissiond by Interreg NOMADe
 *
 */


#ifndef _IMU_REGISTERS_H_
#define _IMU_REGISTERS_H_

// Register description for ICM-20948 IMU

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