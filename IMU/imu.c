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
 *         File: imu.c
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: IMU -> ICM20948
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#include "imu.h"

// Logging functionality
#define NRF_LOG_MODULE_NAME imu
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// I2C
#include "usr_twi.h"
#include "nrf_drv_twi.h"

// GPIO
#include "nrf_drv_gpiote.h"

// Application scheduler
#include "app_scheduler.h"

// FIFO buffers
#include "app_fifo.h"

// IMU
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"


// Invensense Drivers
#include "../Devices/SerifHal.h"
#include "../Devices/HostSerif.h"
#include "../Devices/DeviceIcm20948.h"
#include "../DynamicProtocol/DynProtocol.h"
#include "../DynamicProtocol/DynProtocolTransportUart.h"
#include "../EmbUtils/Message.h"
#include "DataConverter.h"

#include "Device.h"
#include "SensorConfig.h"
#include "SensorConfig.h"


#include "imu_params.h"

// BLE Motion service
#include "ble_tms.h"

#include "usr_ble.h"

// Utilities
#include "usr_util.h"

#include "time_sync.h"
#include "usr_tmr.h"


static bool in_wom = false;
static bool in_shutdown = false;


BUFFER buff;

// Stuct buffer to keep track of latest IMU samples
imu_data_t imu_data;


// Initialisation of IMU struct
extern IMU imu;

/*
 * States for icm20948 device object
 */

static inv_device_icm20948_t device_icm20948; 
static uint8_t dmp3_image[] = {
	#include "../Images/icm20948_img.dmp3a.h"
};

inv_device_t * device; /* just a handy variable to keep the handle to device object */

/* Activity classification */
const char * activityName(int act);

static void check_rc(int rc);

void imu_evt_poll_sceduled(void * p_event_data, uint16_t event_size);
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

extern volatile bool twi_tx_done ;
extern volatile bool twi_rx_done;

extern const nrf_drv_twi_t m_twi;

extern ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);
extern ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop);
extern void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);


#define ICM_20948_I2C_ADDRESS		0x69U


// forward declarations
int my_serif_open_adapter(void);
int my_serif_close_adapter(void);
int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen);
int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen);
int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context);

// definition of the instance
const inv_host_serif_t my_serif_instance = {
        my_serif_open_adapter,
        my_serif_close_adapter,
        my_serif_open_read_reg,
        my_serif_open_write_reg,
		my_adapter_register_interrupt_callback,
        256,
		256,
        INV_HOST_SERIF_TYPE_I2C
};

// Not used - is integrated in I2C read - write
int my_serif_open_adapter(void)
{
				int rc = 0;
				return rc;
}

int my_serif_close_adapter(void)
{
				int rc = 0;
				return rc;
}


int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context)
{
//				(*interrupt_cb) = twi_handler;
			
        int rc=0;
        return rc;
}

int my_serif_open_read_reg(uint8_t reg, uint8_t * rbuffer, uint32_t rlen)
{
	(void)reg, (void)rbuffer, (void)rlen;
	
	ret_code_t error = i2c_read_bytes( &m_twi, ICM_20948_I2C_ADDRESS, reg, rbuffer, rlen);
	if(error == NRF_SUCCESS)
	{
		return 0; // On success
	}else{
		return -1;	// shall return a negative value on error
	}	
}

int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
	(void)reg, (void)wbuffer, (void)wlen;

	ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, reg, wbuffer, wlen, false);

	if(error == NRF_SUCCESS)
	{
		return 0;   // On success
	}else{
		return -1;	// shall return a negative value on error
	}
}


// Timer
#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_timer.h"
#include "bsp.h"
#include "app_error.h"

#include "nrf_delay.h"

extern const nrf_drv_timer_t TIMER_MICROS;

/*
 * Time implementation for Icm20948.
 */
uint64_t inv_icm20948_get_time_us(void)
{	
				uint32_t time_us = nrf_drv_timer_capture(&TIMER_MICROS, NRF_TIMER_CC_CHANNEL0);
//				NRF_LOG_INFO("Timer value requested: %d", time_us);
				return time_us;
}





/* 
 * High resolution sleep implementation for Icm20948.
 * Used at initilization stage. ~100us is sufficient.
 */
void inv_icm20948_sleep_us(int us)
{
        /*
         * You may provide a sleep function that blocks the current programm
         * execution for the specified amount of us
         */
	
        (void)us;
	
				nrf_delay_us(us);
}




/////////// SENSOR EVENTS \\\\\\\\\\\\\\\\\\\\\\

#include "../Devices/SerifHal.h"
#include "../Devices/DeviceIcm20948.h"
#include "../DynamicProtocol/DynProtocol.h"
#include "../DynamicProtocol/DynProtocolTransportUart.h"
// INV_MSG functionality
#include "../EmbUtils/Message.h"


/*
 * Function to return activity name in printable char
 */
const char * activityName(int act)
{
	switch(act) {
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_BEGIN:          return "BEGIN IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_BEGIN:             return "BEGIN WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_BEGIN:             return "BEGIN RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_BEGIN:          return "BEGIN ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_BEGIN:                return "BEGIN TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_BEGIN:               return "BEGIN STILL";
	case INV_SENSOR_BAC_EVENT_ACT_IN_VEHICLE_END:            return "END IN_VEHICLE";
	case INV_SENSOR_BAC_EVENT_ACT_WALKING_END:               return "END WALKING";
	case INV_SENSOR_BAC_EVENT_ACT_RUNNING_END:               return "END RUNNING";
	case INV_SENSOR_BAC_EVENT_ACT_ON_BICYCLE_END:            return "END ON_BICYCLE";
	case INV_SENSOR_BAC_EVENT_ACT_TILT_END:                  return "END TILT";
	case INV_SENSOR_BAC_EVENT_ACT_STILL_END:                 return "END STILL";
	default:                                                 return "unknown activity!";
	}
}

int counterr = 0;
char stringsend[247];

static void check_rc(int rc)
{
	if( rc < 0 )
	{
		ret_code_t err_code = NRF_ERROR_INTERNAL;
		APP_ERROR_CHECK(err_code);
	}

	if(rc == -1) {
		NRF_LOG_INFO("BAD RC=%d", rc);
		NRF_LOG_FLUSH();
		while(1);
	}
}

uint8_t test_troughput_array[132];



/*
 * Callback called upon sensor event reception
 * This function is called in the same function than inv_device_poll()
 */
static void sensor_event_cb(const inv_sensor_event_t * event, void * arg)
{
        /* arg will contained the value provided at init time */
        (void)arg;
        
        (void)event;
        /* ... do something with event */
	
				// NRF_LOG_INFO("Sensor event!");
				//NRF_LOG_FLUSH();	
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
		
		// Temp
// imu_send_data();
		
		ret_code_t err_code;
		size_t len_in;
		
		uint8_t config_data[1];
		
		counterr++;


	switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {

		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
		{
//			NRF_LOG_INFO("data event: %d %d %d %d", //inv_sensor_str(event->sensor),
//					(int)event->timestamp,
//					(int)event->data.raw3d.vect[0],
//					(int)event->data.raw3d.vect[1],
//					(int)event->data.raw3d.vect[2]);
				NRF_LOG_INFO("Data");
//					NRF_LOG_INFO("%d	%d	%d", event->data.raw3d.vect[0], event->data.raw3d.vect[1], event->data.raw3d.vect[2]);
			break;
		}
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
		{
			// NRF_LOG_INFO("data event %s (mg): %d %d %d %d", inv_sensor_str(event->sensor),
			// 		(int)(event->data.acc.vect[0]*1000),
			// 		(int)(event->data.acc.vect[1]*1000),
			// 		(int)(event->data.acc.vect[2]*1000),
			// 		(int)(event->data.acc.accuracy_flag));
					
				#ifdef USE_NUS
				// Put data in the ringbuffer
				config_data[0] = ENABLE_ACCEL;
				len_in = sizeof(config_data);

				uint32_t buffer_acc_len = sizeof(config_data) + sizeof(event->data.acc.vect);
				uint8_t buffer_acc[buffer_acc_len];

				// Copy data into buffer
				memcpy(buffer_acc, config_data, sizeof(config_data));
				memcpy(&buffer_acc[1], (event->data.acc.vect), sizeof(event->data.acc.vect));

				// Put the data in FIFO buffer: APP_FIFO instead of ringbuff library
            	err_code = app_fifo_write(&buff.imu_fifo, buffer_acc, &buffer_acc_len);
				if (err_code == NRF_ERROR_NO_MEM)
            	{
                	NRF_LOG_INFO("IMU FIFO BUFFER FULL!");
            	}
				if (err_code == NRF_SUCCESS)
				{
					NRF_LOG_INFO("OK");
				}
				#endif

				// Save latest data in buffer
				imu_data.accel.x =      (int16_t)((event->data.acc.vect[0]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
				imu_data.accel.y =      (int16_t)((event->data.acc.vect[1]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));
				imu_data.accel.z =      (int16_t)((event->data.acc.vect[2]) * (1 << RAW_Q_FORMAT_ACC_COMMA_BITS));

			break;
		}
		case INV_SENSOR_TYPE_GYROSCOPE:
		{
			// NRF_LOG_INFO("data event %s (mdps): %d %d %d %d", inv_sensor_str(event->sensor),
			// 		(int)(event->data.gyr.vect[0]*1000),
			// 		(int)(event->data.gyr.vect[1]*1000),
			// 		(int)(event->data.gyr.vect[2]*1000),
			// 		(int)(event->data.gyr.accuracy_flag));
					
				#ifdef USE_NUS
				// Put data in the ringbuffer
				config_data[0] = ENABLE_GYRO;
				len_in = sizeof(config_data);

				uint32_t buffer_gyro_len = sizeof(config_data) + sizeof(event->data.gyr.vect);
				uint8_t buffer_gyro[buffer_gyro_len];

				// Copy data into buffer
				memcpy(buffer_gyro, config_data, sizeof(config_data));
				memcpy(&buffer_gyro[1], (event->data.gyr.vect), sizeof(event->data.gyr.vect));

				// Put the data in FIFO buffer: APP_FIFO instead of ringbuff library
            	err_code = app_fifo_write(&buff.imu_fifo, buffer_gyro, &buffer_gyro_len);
				if (err_code == NRF_ERROR_NO_MEM)
            	{
                	NRF_LOG_INFO("IMU FIFO BUFFER FULL!");
            	}
				if (err_code == NRF_SUCCESS)
				{
					NRF_LOG_INFO("OK");
				}
				#endif

				// Save latest data in buffer
				imu_data.gyro.x =       (int16_t)((event->data.gyr.vect[0]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
				imu_data.gyro.y =       (int16_t)((event->data.gyr.vect[1]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));
				imu_data.gyro.z =       (int16_t)((event->data.gyr.vect[2]) * (1 << RAW_Q_FORMAT_GYR_COMMA_BITS));

			break;
		}
		case INV_SENSOR_TYPE_MAGNETOMETER:
		{
			// NRF_LOG_INFO("data event %s (nT): %d %d %d %d", inv_sensor_str(event->sensor),
			// 		(int)(event->data.mag.vect[0]*1000),
			// 		(int)(event->data.mag.vect[1]*1000),
			// 		(int)(event->data.mag.vect[2]*1000),
			// 		(int)(event->data.mag.accuracy_flag));
					
				#ifdef USE_NUS
				// Put data in the ringbuffer
				config_data[0] = ENABLE_MAG;
				len_in = sizeof(config_data);

				uint32_t buffer_mag_len = sizeof(config_data) + sizeof(event->data.mag.vect);
				uint8_t buffer_mag[buffer_mag_len];

				// Copy data into buffer
				memcpy(buffer_mag, config_data, sizeof(config_data));
				memcpy(&buffer_mag[1], (event->data.mag.vect), sizeof(event->data.mag.vect));

				// Put the data in FIFO buffer: APP_FIFO instead of ringbuff library
            	err_code = app_fifo_write(&buff.imu_fifo, buffer_mag, &buffer_mag_len);
				if (err_code == NRF_ERROR_NO_MEM)
            	{
                	NRF_LOG_INFO("IMU FIFO BUFFER FULL!");
            	}
				if (err_code == NRF_SUCCESS)
				{
					NRF_LOG_INFO("OK");
				}
				#endif

				// Save latest data in buffer
				imu_data.mag.y =   -(int16_t)((event->data.mag.vect[0]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes and inverted. Corrected for rotation of axes.
				imu_data.mag.x =    (int16_t)((event->data.mag.vect[1]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS)); // Changed axes. Corrected for rotation of axes.
				imu_data.mag.z =    (int16_t)((event->data.mag.vect[2]) * (1 << RAW_Q_FORMAT_CMP_COMMA_BITS));
					
			break;
		}
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
		{
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		}
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
		{
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		}
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		{			
			// NRF_LOG_INFO("%s:	%d %d %d %d", inv_sensor_str(event->sensor),
			// 		(int)(event->data.quaternion.quat[0]*1000),
			// 		(int)(event->data.quaternion.quat[1]*1000),
			// 		(int)(event->data.quaternion.quat[2]*1000),
			// 		(int)(event->data.quaternion.quat[3]*1000));
//					(int)(event->data.gyr.accuracy_flag),
//					(int)(event->data.acc.accuracy_flag),	
//					(int)(event->data.mag.accuracy_flag), // 0 - 3: not calibrated - fully calibrated
//					(int)(event->data.quaternion.accuracy_flag));
				



				#ifdef USE_NUS
				// Put data in the ringbuffer
				config_data[0] = ENABLE_QUAT6;
				len_in = sizeof(config_data);

				uint32_t buffer_len = sizeof(config_data) + sizeof(event->data.quaternion.quat);
				uint8_t buffer[buffer_len];

				// Copy data into buffer
				memcpy(buffer, config_data, sizeof(config_data));
				memcpy(&buffer[1], (event->data.quaternion.quat), sizeof(event->data.quaternion.quat));

				
				// Put the data in FIFO buffer: APP_FIFO instead of ringbuff library
            	err_code = app_fifo_write(&buff.imu_fifo, buffer, &buffer_len);
				if (err_code == NRF_ERROR_NO_MEM)
            	{
                	NRF_LOG_INFO("IMU FIFO BUFFER FULL!");
            	}
				if (err_code == NRF_SUCCESS)
				{
					NRF_LOG_INFO("OK");
				}
				#endif

				float quat[4];
				memcpy(quat, (event->data.quaternion.quat), sizeof(event->data.quaternion.quat));

				fixed_point_t p_quat[4];
				p_quat[0] = float_to_fixed_quat(quat[0]);
				p_quat[1] = float_to_fixed_quat(quat[1]);
				p_quat[2] = float_to_fixed_quat(quat[2]);
				p_quat[3] = float_to_fixed_quat(quat[3]);

				imu_data.quat.w = p_quat[0];
				imu_data.quat.x = p_quat[1];
				imu_data.quat.y = p_quat[2];
				imu_data.quat.z = p_quat[3];

				break;
			}
		case INV_SENSOR_TYPE_ORIENTATION:
		{
//			NRF_LOG_INFO("data event %s (e-3):, %d, %d, %d, Accuracy: %d ", inv_sensor_str(event->sensor),
//					(int)(event->data.orientation.x*1000),
//					(int)(event->data.orientation.y*1000),
//					(int)(event->data.orientation.z*1000),
//					(int)(event->data.orientation.accuracy_flag*1000)); // 0 - 3: not calibrated - fully calibrated
		NRF_LOG_INFO("%s:	%d %d %d", // rewritten write funtion to allow easier plotting
					inv_sensor_str(event->sensor),
					(int)(event->data.orientation.x),
					(int)(event->data.orientation.y),
					(int)(event->data.orientation.z));
//					(int)(event->data.orientation.accuracy_flag),
					// (int)(event->data.gyr.accuracy_flag),
					// (int)(event->data.acc.accuracy_flag),	
					// (int)(event->data.mag.accuracy_flag)); // 0 - 3: not calibrated - fully calibrated
					

			#ifdef USE_NUS		
				// 	float buffer_orientation[3];
				// 	buffer_orientation[0] = event->data.orientation.x;
				// 	buffer_orientation[1] = event->data.orientation.y;
				// 	buffer_orientation[2] = event->data.orientation.z;
					
				// // Put data in the ringbuffer
				// len_in = sizeof(buffer_orientation);					
				// err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(buffer_orientation), &len_in); //(uint8_t *)(event->data.quaternion.quat)
				// APP_ERROR_CHECK(err_code);
			#endif

			fixed_point_t p_euler[3];

			float euler[3];
			euler[0] = event->data.orientation.x;
			euler[1] = event->data.orientation.y;
			euler[2] = event->data.orientation.z;

			p_euler[0] = float_to_fixed_euler(euler[0]);
			p_euler[1] = float_to_fixed_euler(euler[1]);
			p_euler[2] = float_to_fixed_euler(euler[2]);

            imu_data.euler.yaw   = p_euler[0];
            imu_data.euler.pitch  = p_euler[1];
            imu_data.euler.roll    = p_euler[2];
					
			break;
		}
		case INV_SENSOR_TYPE_BAC:
		{
//			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : %d %s", inv_sensor_str(event->sensor),
//					event->data.bac.event, activityName(event->data.bac.event));
				NRF_LOG_INFO("data event %s : %d %s", inv_sensor_str(event->sensor),
					event->data.bac.event, activityName(event->data.bac.event));
			break;
		case INV_SENSOR_TYPE_STEP_COUNTER:
			NRF_LOG_INFO("data event %s : %lu", inv_sensor_str(event->sensor),
					(unsigned long)event->data.step.count);
			break;
		case INV_SENSOR_TYPE_PICK_UP_GESTURE:
				NRF_LOG_INFO("PICKUP GESTURE");
				// If we wake on motion, we need to start advertising again to be able to retrieve the BLE device
				advertising_start();
				break;
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
				NRF_LOG_INFO("Wake on motion!");

				// If we wake on motion, we need to start advertising again to be able to retrieve the BLE device
				advertising_start();
			break;
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			NRF_LOG_INFO("DEFAULT");
			// INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
		}
	}
}
}

/*
 * A listener onject will handle sensor events
 */
inv_sensor_listener_t sensor_listener = {
        sensor_event_cb, /* callback that will receive sensor events */
        (void *)0xDEAD   /* some pointer passed to the callback */
};


/*
 * Printer function for IDD message facility
 */
void msg_printer(int level, const char * str, va_list ap)
{
	NRF_LOG_INFO(str);
	NRF_LOG_FLUSH();
}


/**
 * @}
 */


//void apply_stored_offsets(void)
//{
//	uint8_t sensor_bias[84];
//	int32_t acc_bias_q16[6] = {0}, gyro_bias_q16[6] = {0};
//	uint8_t i, idx = 0;
//	int rc;
//	
//	
//	// TODO own implementation needed
//	/* Retrieve Sel-test offsets stored in NV memory */
////	if(flash_manager_readData(sensor_bias) != 0) {
////		INV_MSG(INV_MSG_LEVEL_WARNING, "No bias values retrieved from NV memory !");
////		return;
////	}
//	
//	for(i = 0; i < 6; i++)
//		gyro_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[i * sizeof(uint32_t)]));
//	idx += sizeof(gyro_bias_q16);
//	rc = inv_device_set_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,
//		VSENSOR_CONFIG_TYPE_OFFSET, gyro_bias_q16, sizeof(gyro_bias_q16));
//	check_rc(rc);
//	
//	for(i = 0; i < 6; i++)
//		acc_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[idx + i * sizeof(uint32_t)]));
//	idx += sizeof(acc_bias_q16);
//	rc = inv_device_set_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER,
//		VSENSOR_CONFIG_TYPE_OFFSET, acc_bias_q16, sizeof(acc_bias_q16));

//}



//void store_offsets(void)
//{
//	int rc = 0;
//	uint8_t i, idx = 0;
//	int gyro_bias_q16[6] = {0}, acc_bias_q16[6] = {0};

//	uint8_t sensor_bias[84] = {0};
//	
//	/* Strore Self-test bias in NV memory */
//	rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,
//			VSENSOR_CONFIG_TYPE_OFFSET, gyro_bias_q16, sizeof(gyro_bias_q16));
//	check_rc(rc);
//	for(i = 0; i < 6; i++)
//		inv_dc_int32_to_little8(gyro_bias_q16[i], &sensor_bias[i * sizeof(uint32_t)]);
//	idx += sizeof(gyro_bias_q16);
//	
//	rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER,
//			VSENSOR_CONFIG_TYPE_OFFSET, acc_bias_q16, sizeof(acc_bias_q16));
//	check_rc(rc);
//	for(i = 0; i < 6; i++)
//		inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
//	idx += sizeof(acc_bias_q16);

//	// TODO own implementation needed to store sensor_bias in non volatile memory
//	// flash_manager_writeData(sensor_bias);
//}




bool imu_in_shutdown(void)
{
	return in_shutdown;
}

void imu_set_in_shutdown(bool enable)
{
	in_shutdown = enable;
}


bool imu_in_wom(void)
{
	return in_wom;
}

void imu_set_in_wom(bool enable)
{
	in_wom = enable;
}


/* Interrupt pin handeler callback function */
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		ret_code_t err_code;

		// Temp disable interrupts in transition period
		if(!imu_in_shutdown())
		{
			if(pin == INT_PIN)
			{
					NRF_LOG_INFO("IMU INTERRUPT");

					// If there are already events in the queue
					if(imu.evt_scheduled > 0)
					{
						imu.evt_scheduled++;
					}
					// If there are not yet any events in the queue, schedule event. In gpiote_evt_sceduled all callbacks are called
					else
					{
						imu.evt_scheduled++;
						err_code = app_sched_event_put(0, 0, imu_evt_poll_sceduled);
						APP_ERROR_CHECK(err_code);
					}
			}
		}
}



// Event handler DIY
/**@brief GPIOTE sceduled handler, executed in main-context.
 */
void imu_evt_poll_sceduled(void * p_event_data, uint16_t event_size)
{
    while ( (imu.evt_scheduled > 0) )//&& m_mpu9250.enabled) TODO check when IMU is enabled or not
    {
		// nrf_gpio_pin_set(PIN_IMU_ACTIVITY);

		if(imu_in_wom())
		{
			NRF_LOG_INFO("WoM wake-up");

			// Init IMU + reset device
			// Keep track of WoM state
			imu_set_in_wom(false);

			imu_timers_init();

			// Initialize IMU with DMP (Invensense Driver)
			NRF_LOG_INFO("imu_re_init start");
			imu_re_init();
			NRF_LOG_INFO("imu_re_init completed");

			// Start advertising again
			advertising_start();
			NRF_LOG_INFO("advertising_start");

			// Disable time synchronization
			TimeSync_enable();

		}else{
			// Poll all data from IMU
			inv_device_poll(device);
		}

				// 		uint8_t temp[1];
				// ICM_20948_registerRead(ICM_20948_REG_INT_STATUS, 1, temp);
				// NRF_LOG_INFO("interrupt source: 0x%X - %d", temp[0], temp[0]);
				// NRF_LOG_FLUSH();

		// nrf_gpio_pin_clear(PIN_IMU_ACTIVITY);
		imu.evt_scheduled--;
    }
}



ret_code_t imu_enable_sensors(IMU imu)
{
		int rc = 0;

		if(imu.wom)
		{
			NRF_LOG_INFO("Start WOM");
			rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_SMD);
			check_rc(rc);
			rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_SMD, 0);
			check_rc(rc);
			rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_SMD);
			check_rc(rc);
		}
		else if(!imu.wom)
		{
			NRF_LOG_INFO("Stop WOM");
			rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_SMD);
			check_rc(rc);
		}

		if(imu.stop)
		{
		// Stop all sensors before enabling the new ones
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);	
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);	
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);	
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		}
		// If enabled: Start 6DoF quaternion output
		if(imu.quat6_enabled)
		{
		NRF_LOG_INFO("Start QUAT6");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		check_rc(rc);
		}
		else if(!imu.quat6_enabled)
		{
		NRF_LOG_INFO("Stop QUAT6");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		check_rc(rc);
		}
		// If enabled: Start 9DoF quaternion output
		if(imu.quat9_enabled)
		{
		NRF_LOG_INFO("Start QUAT9");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ROTATION_VECTOR, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		}
		if(!imu.quat9_enabled)
		{
		NRF_LOG_INFO("Stop QUAT9");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		}
		// If enabled: Start Euler angles
		if(imu.euler_enabled)
		{
		NRF_LOG_INFO("Start 9DoF EULER");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ORIENTATION, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);
		}
		if(!imu.euler_enabled)
		{
		NRF_LOG_INFO("Stop 9DoF EULER");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Gyroscope output
		if(imu.gyro_enabled)
		{
		NRF_LOG_INFO("Start GYRO");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);
		}
		if(!imu.gyro_enabled)
		{
		NRF_LOG_INFO("Stop GYRO");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Accelerometer output
		if(imu.accel_enabled)
		{
		NRF_LOG_INFO("Start ACCEL");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);
		}
		if(!imu.accel_enabled)
		{
		NRF_LOG_INFO("Stop ACCEL");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Accelerometer output
		if(imu.mag_enabled)
		{
		NRF_LOG_INFO("Start MAG");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, IMU_DEFAULT_SAMPL_FREQ);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		}
		if(!imu.mag_enabled)
		{
		NRF_LOG_INFO("Stop MAG");
		rc += inv_device_stop_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		}
		
		return NRF_SUCCESS;
}

static void imu_buff_init()
{
	ret_code_t err_code;

	// Initialize IMU FIFO structure
	err_code = app_fifo_init(&buff.imu_fifo, buff.imu_fifo_buff, (uint16_t)sizeof(buff.imu_fifo_buff));
	APP_ERROR_CHECK(err_code);

	// Initialize QUAT FIFO structure
	err_code = app_fifo_init(&buff.quat_fifo, buff.quat_fifo_buff, (uint16_t)sizeof(buff.quat_fifo_buff));
	APP_ERROR_CHECK(err_code);

	// Initialize RAW FIFO structure
	err_code = app_fifo_init(&buff.raw_fifo, buff.raw_fifo_buff, (uint16_t)sizeof(buff.raw_fifo_buff));
	APP_ERROR_CHECK(err_code);
}

void imu_clear_buff()
{
	ret_code_t err_code;

	// Clear all data in buffers
	err_code = app_fifo_flush(&buff.imu_fifo);
	APP_ERROR_CHECK(err_code);

	err_code = app_fifo_flush(&buff.quat_fifo);
	APP_ERROR_CHECK(err_code);

	err_code = app_fifo_flush(&buff.raw_fifo);
	APP_ERROR_CHECK(err_code);
}

void imu_power_en(bool enable)
{
	ret_code_t err_code;

	if(enable)
	{
		// Set pin modes - set high drive strength
		nrf_gpio_cfg(IMU_POWER_PIN, NRF_GPIO_PIN_DIR_OUTPUT, NRF_GPIO_PIN_INPUT_DISCONNECT, NRF_GPIO_PIN_NOPULL, NRF_GPIO_PIN_H0H1, NRF_GPIO_PIN_NOSENSE);
		// Set pin high to enable IMU
		nrf_gpio_pin_set(IMU_POWER_PIN);
		err_code = NRF_SUCCESS;
	}
	else if(!enable)
	{
		// Set pin low to disable IMU
		nrf_gpio_pin_clear(IMU_POWER_PIN);
		// Set to default pin configuration: input with no pull resistors
		nrf_gpio_cfg_default(IMU_POWER_PIN);
		err_code = NRF_SUCCESS;
	}
	else
	{
		err_code = NRF_ERROR_INVALID_PARAM;
	}
	APP_ERROR_CHECK(err_code);

	// Some delay - may not be necessary
	nrf_delay_ms(100);
}


void imu_init(void)
{
#if IMU_ENABLED == 1

		NRF_LOG_INFO("IMU Init");

		// Power on the IMU
		imu_power_en(true);

		// Initialize necessary buffers for data transmission
		imu_buff_init();

		/*
		 * Setup message facility to see internal traces from IDD
		 */
		INV_MSG_SETUP(MSG_LEVEL, msg_printer);

		INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
		INV_MSG(INV_MSG_LEVEL_INFO, "#          20948 example          #");
		INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
		NRF_LOG_FLUSH();

		/* To keep track of errors */
		int rc = 0;
		
		uint8_t whoami;
		
		/* Open serial interface (SPI or I2C) before playing with the device */
		// Not needed anymore - this is implemented in the inv_host_serif_open(&my_serif_instance)
//		twi_init();
		// rc = inv_host_serif_open(&my_serif_instance);
		// check_rc(rc);
		
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
		
		/*
		 * Create ICM20948 Device 
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
		inv_device_icm20948_init(&device_icm20948, &my_serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
		
		/*
		 * Simply get generic device handle from Icm20948 Device
		 */
		device = inv_device_icm20948_get_base(&device_icm20948);
		NRF_LOG_FLUSH();
		
		/* Just get the whoami */
		rc += inv_device_whoami(device, &whoami);
		check_rc(rc);
		NRF_LOG_INFO("Data: 0x%x", whoami);
		NRF_LOG_FLUSH();

		/* Configure and initialize the Icm20948 device */
		NRF_LOG_INFO("Setting up ICM20948");
		NRF_LOG_FLUSH();
		rc += inv_device_setup(device);
		check_rc(rc);
		
		rc += inv_device_load(device, (int) NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, (int) NULL);
		check_rc(rc);

		NRF_LOG_INFO("DMP Image loaded");
		NRF_LOG_FLUSH();
#endif
}
void imu_re_init(void)
{
#if IMU_ENABLED == 1

		NRF_LOG_INFO("IMU Init");

		// Power on the IMU
		// imu_power_en(true);

		// Initialize necessary buffers for data transmission
		// imu_buff_init();

		/*
		 * Setup message facility to see internal traces from IDD
		 */
		INV_MSG_SETUP(MSG_LEVEL, msg_printer);

		INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
		INV_MSG(INV_MSG_LEVEL_INFO, "#          20948 example          #");
		INV_MSG(INV_MSG_LEVEL_INFO, "###################################");
		NRF_LOG_FLUSH();

		/* To keep track of errors */
		int rc = 0;
		
		uint8_t whoami;
		
		/* Open serial interface (SPI or I2C) before playing with the device */
		// Not needed anymore - this is implemented in the inv_host_serif_open(&my_serif_instance)
//		twi_init();
		// rc = inv_host_serif_open(&my_serif_instance);
		// check_rc(rc);
		
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
		
		/*
		 * Create ICM20948 Device 
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
		// inv_device_icm20948_init(&device_icm20948, &my_serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
		
		/*
		 * Simply get generic device handle from Icm20948 Device
		 */
		// device = inv_device_icm20948_get_base(&device_icm20948);
		// NRF_LOG_FLUSH();
		
		/* Just get the whoami */
		rc += inv_device_whoami(device, &whoami);
		check_rc(rc);
		NRF_LOG_INFO("Data: 0x%x", whoami);
		NRF_LOG_FLUSH();

		/* Configure and initialize the Icm20948 device */
		NRF_LOG_INFO("Setting up ICM20948");
		NRF_LOG_FLUSH();
		rc += inv_device_setup(device);
		check_rc(rc);
		
		rc += inv_device_load(device, (int) NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, (int) NULL);
		check_rc(rc);

		NRF_LOG_INFO("DMP Image loaded");
		NRF_LOG_FLUSH();
#endif
}






uint8_t number_of_quat_packets = 0;
uint8_t number_of_raw_packets = 0;




void imu_send_data()
{
	ret_code_t err_code;

	if(imu.quat6_enabled || imu.quat9_enabled)
	{
		ble_tms_single_quat_t single_quat;
		uint32_t single_quat_len = sizeof(single_quat);

		// Put data in send buffer + increment packet counter
		number_of_quat_packets++;

		// Get latest available data
		single_quat.w = imu_data.quat.w;
		single_quat.x = imu_data.quat.x;
		single_quat.y = imu_data.quat.y;
		single_quat.z = imu_data.quat.z;

		// Put data in send buffer
		err_code = app_fifo_write(&buff.quat_fifo, (uint8_t *) &single_quat, &single_quat_len);
		if (err_code == NRF_ERROR_NO_MEM)
		{
			NRF_LOG_INFO("QUAT FIFO BUFFER FULL!");
		}

		// If 10 packets are queued, send them out
		if( number_of_quat_packets >= BLE_PACKET_BUFFER_COUNT)
		{		
			ble_tms_quat_t data;
			uint32_t data_len = sizeof(data);

			err_code = app_fifo_read(&buff.quat_fifo, (uint8_t *) &data, &data_len);
			if (err_code == NRF_ERROR_NO_MEM)
			{
				NRF_LOG_INFO("QUAT FIFO BUFFER FULL!");
			}

			// Send data over BLE
			ble_send_quat(&data);

			number_of_quat_packets = 0;
			// NRF_LOG_INFO("quat set");
		}

	}
	if(imu.gyro_enabled || imu.accel_enabled || imu.mag_enabled)
	{
		ble_tms_single_raw_t single_raw;
		uint32_t single_raw_len = sizeof(single_raw);

		// Put data in send buffer + increment packet counter
		number_of_raw_packets++;

		// Get latest available data
		// Get last gyro data from buffer
		single_raw.gyro.x = imu_data.gyro.x;
		single_raw.gyro.y = imu_data.gyro.y;
		single_raw.gyro.z = imu_data.gyro.z;

		// Get last accel data from buffer
		single_raw.accel.x = imu_data.accel.x;
		single_raw.accel.y = imu_data.accel.y;
		single_raw.accel.z = imu_data.accel.z;

		// Get last mag data from buffer
		single_raw.compass.x = imu_data.mag.x;
		single_raw.compass.y = imu_data.mag.y;
		single_raw.compass.z = imu_data.mag.z;

		// Put data in send buffer
		err_code = app_fifo_write(&buff.raw_fifo, (uint8_t *) &single_raw, &single_raw_len);
		if (err_code == NRF_ERROR_NO_MEM)
		{
			NRF_LOG_INFO("QUAT FIFO BUFFER FULL!");
		}

		// If 10 packets are queued, send them out
		if( number_of_raw_packets >= BLE_PACKET_BUFFER_COUNT)
		{		
			ble_tms_raw_t data;
			uint32_t data_len = sizeof(data);

			err_code = app_fifo_read(&buff.raw_fifo, (uint8_t *) &data, &data_len);
			if (err_code == NRF_ERROR_NO_MEM)
			{
				NRF_LOG_INFO("QUAT FIFO BUFFER FULL!");
			}

			// Send data over BLE
			ble_send_raw(&data);

			number_of_raw_packets = 0;
			// NRF_LOG_INFO("quat set");
		}
	}
	if(imu.euler_enabled)
	{
		ble_tms_euler_t data;

		data.yaw = imu_data.euler.yaw;
		data.pitch = imu_data.euler.pitch;
		data.roll = imu_data.euler.roll;

		// Send data over BLE
		ble_send_euler(&data);

	}
	if(imu.adc)
	{
		ble_tms_adc_t data;

		data.raw[0] = 1;

		ble_send_adc(&data);

	}
}


// void imu_get_config()
// {
// 	int rc = 0;
		
// 	rc += inv_device_get_sensor_config(device, );
// 	check_rc(rc);
// }

void imu_set_config()
{
	int rc = 0;

	// 0 -> low power mode
	// 1 -> low noise mode
	inv_sensor_config_powermode_t power_mode;
	power_mode.lowpower_or_highperformance = 0;
    
	// rc = inv_device_set_sensor_config(device, 0, INV_SENSOR_CONFIG_POWER_MODE, &power_mode, 1);
	// check_rc(rc);

	NRF_LOG_INFO("Ping SMD");
	rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_SMD);
	check_rc(rc);

	NRF_LOG_INFO("Set SMD period");
	rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_SMD, 0);
	check_rc(rc);

	NRF_LOG_INFO("Start SMD");
	rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_SMD);
	check_rc(rc);

	// dmp_icm20948_set_wom_enable();
}


void imu_wom_enable(bool enable)
{
	int rc = 0;

	rc += inv_device_set_running_state(device, false);
	check_rc(rc);

	// rc += inv_device_set_sensor_config();
	// check_rc(rc);
}


void imu_deinit()
{
	int rc = 0;

	// Reset device to be in a known state
	rc += inv_device_reset(device);
	check_rc(rc);

	// Shutdown IMU and clear internal state
	rc += inv_device_cleanup(device);
	check_rc(rc);

	// rc = inv_host_serif_close(&my_serif_instance);
	// check_rc(rc);

	// Power on the IMU
	// imu_power_en(false);
}

void ICM20948_reset()
{
/* Set H_RESET bit to initiate soft reset */
ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, ICM_20948_BIT_H_RESET);

/* Wait 100ms to complete the reset sequence */
nrf_delay_ms(100);
}

void ICM_20948_bankSelect(uint8_t bank)
{
	uint8_t wBuffer[2];
	wBuffer[0] = ICM_20948_REG_BANK_SEL;
	wBuffer[1] = (bank << 4);

	uint8_t temp[1];
	memcpy(temp, &wBuffer[1], 1);

	ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, wBuffer[0], temp, 1, false);
	APP_ERROR_CHECK(error);
}

void ICM_20948_registerWrite(uint16_t addr, uint8_t data)
{
	uint8_t regAddr;
	uint8_t bank;

	regAddr = (uint8_t) (addr & 0x7F);
	bank = (uint8_t) (addr >> 7);

	ICM_20948_bankSelect(bank);
	
	uint8_t temp[1];

	memcpy(temp, &data, 1);

	// int rc = inv_device_write_mems_register(device, 0, addr, temp, 1);
	// check_rc(rc);

	ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, regAddr, temp, 1, false);
	APP_ERROR_CHECK(error);

	return;
}
void ICM_20948_registerRead(uint16_t addr, int numBytes, uint8_t *data)
{
	// int rc = inv_device_read_mems_register(device, 0, addr, data, numBytes);
	// check_rc(rc);

	ret_code_t error = i2c_read_bytes( &m_twi, ICM_20948_I2C_ADDRESS, addr, data, 1);
	APP_ERROR_CHECK(error);

	return;
}


uint32_t ICM_20948_sensorEnable(bool accel, bool gyro, bool temp)
{
  uint8_t pwrManagement1;
  uint8_t pwrManagement2;

  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &pwrManagement1);
  pwrManagement2 = 0;

  /* To enable the accelerometer clear the DISABLE_ACCEL bits in PWR_MGMT_2 */
  if ( accel ) {
    pwrManagement2 &= ~(ICM_20948_BIT_PWR_ACCEL_STBY);
  } else {
    pwrManagement2 |= ICM_20948_BIT_PWR_ACCEL_STBY;
  }

  /* To enable gyro clear the DISABLE_GYRO bits in PWR_MGMT_2 */
  if ( gyro ) {
    pwrManagement2 &= ~(ICM_20948_BIT_PWR_GYRO_STBY);
  } else {
    pwrManagement2 |= ICM_20948_BIT_PWR_GYRO_STBY;
  }

  /* To enable the temperature sensor clear the TEMP_DIS bit in PWR_MGMT_1 */
  if ( temp ) {
    pwrManagement1 &= ~(ICM_20948_BIT_TEMP_DIS);
  } else {
    pwrManagement1 |= ICM_20948_BIT_TEMP_DIS;
  }

  /* Write back the modified values */
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, pwrManagement1);
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_2, pwrManagement2);

  return NRF_SUCCESS;
}

uint32_t ICM_20948_sleepModeEnable(bool enable)
{
  uint8_t reg;

  /* Read the Sleep Enable register */
  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &reg);

  if ( enable ) {
    /* Sleep: set the SLEEP bit */
    reg |= ICM_20948_BIT_SLEEP;
  } else {
    /* Wake up: clear the SLEEP bit */
    //reg &= ~(ICM_20948_BIT_SLEEP); /* this was the provided code */

	/* My own solution */
	/* AND to define the bits that can be changed: here bit nr 6 */
	reg &= 0b10111111;
	/*OR met 0 to set the SLEEP bit to 0, not really necessary in this case */
	reg |= 0b00000000;
  }

  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, reg);


  return NRF_SUCCESS;
}

uint32_t ICM_20948_cycleModeEnable(bool enable)
{
  uint8_t reg;

  reg = 0x00;

  if ( enable ) {
    reg = ICM_20948_BIT_ACCEL_CYCLE | ICM_20948_BIT_GYRO_CYCLE;
  }

  ICM_20948_registerWrite(ICM_20948_REG_LP_CONFIG, reg);

  return NRF_SUCCESS;
}

uint32_t ICM_20948_lowPowerModeEnter(bool enAccel, bool enGyro, bool enTemp)
{
  uint8_t data;

  ICM_20948_registerRead(ICM_20948_REG_PWR_MGMT_1, 1, &data);

  if ( enAccel || enGyro || enTemp ) {
    /* Make sure that the chip is not in sleep */
    ICM_20948_sleepModeEnable(false);

    /* And in continuous mode */
    ICM_20948_cycleModeEnable(false);

    /* Enable the accelerometer and the gyroscope*/
    ICM_20948_sensorEnable(enAccel, enGyro, enTemp);
    nrf_delay_ms(50);

    /* Enable cycle mode */
    ICM_20948_cycleModeEnable(true);

    /* Set the LP_EN bit to enable low power mode */
    data |= ICM_20948_BIT_LP_EN;
  } else {
    /* Enable continuous mode */
    ICM_20948_cycleModeEnable(false);

    /* Clear the LP_EN bit to disable low power mode */
    data &= ~ICM_20948_BIT_LP_EN;
  }

  /* Write the updated value to the PWR_MGNT_1 register */
  ICM_20948_registerWrite(ICM_20948_REG_PWR_MGMT_1, data);

  return NRF_SUCCESS;
}

uint32_t ICM_20948_interruptEnable(bool dataReadyEnable, bool womEnable)
{
  uint8_t intEnable;

  /* All interrupts disabled by default */
  intEnable = 0;

  /* Enable one or both of the interrupt sources if required */
  if ( womEnable ) {
    intEnable = ICM_20948_BIT_WOM_INT_EN;
  }
  /* Write value to register */
  ICM_20948_registerWrite(ICM_20948_REG_INT_ENABLE, intEnable);

//   uint8_t temp[2];
// ICM_20948_registerRead(ICM_20948_REG_INT_ENABLE, 1, &temp[0]);

  /* All interrupts disabled by default */
  intEnable = 0;

  if ( dataReadyEnable ) {
    intEnable = ICM_20948_BIT_RAW_DATA_0_RDY_EN;
  }

  /* Write value to register */
  ICM_20948_registerWrite(ICM_20948_REG_INT_ENABLE_1, intEnable);


// ICM_20948_registerRead(ICM_20948_REG_INT_ENABLE_1, 1, &temp[1]);


  return NRF_SUCCESS;
}

uint32_t ICM_20948_accelBandwidthSet(uint8_t accelBw)
{
  uint8_t reg;

  /* Read the GYRO_CONFIG_1 register */
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_CONFIG, 1, &reg);
  reg &= ~(ICM_20948_MASK_ACCEL_BW);

  /* Write the new bandwidth value to the gyro config register */
  reg |= (accelBw & ICM_20948_MASK_ACCEL_BW);
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_CONFIG, reg);

  return NRF_SUCCESS;
}

uint32_t ICM_20948_accelFullscaleSet(uint8_t accelFs)
{
  uint8_t reg;

  accelFs &= ICM_20948_MASK_ACCEL_FULLSCALE;
  ICM_20948_registerRead(ICM_20948_REG_ACCEL_CONFIG, 1, &reg);
  reg &= ~(ICM_20948_MASK_ACCEL_FULLSCALE);
  reg |= accelFs;
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_CONFIG, reg);

  return NRF_SUCCESS;
}

uint32_t ICM_20948_sampleRateSet(float sampleRate)
{
//   ICM_20948_gyroSampleRateSet(sampleRate);
  ICM_20948_accelSampleRateSet(sampleRate);

  return NRF_SUCCESS;
}

float ICM_20948_accelSampleRateSet(float sampleRate)
{
  uint16_t accelDiv;
  float accelSampleRate;

  /* Calculate the sample rate divider */
  accelSampleRate = (1125.0 / sampleRate) - 1.0;

  /* Check if it fits in the divider registers */
  if ( accelSampleRate > 4095.0 ) {
    accelSampleRate = 4095.0;
  }

  if ( accelSampleRate < 0 ) {
    accelSampleRate = 0.0;
  }

  /* Write the value to the registers */
  accelDiv = (uint16_t) accelSampleRate;
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_SMPLRT_DIV_1, (uint8_t) (accelDiv >> 8) );
  ICM_20948_registerWrite(ICM_20948_REG_ACCEL_SMPLRT_DIV_2, (uint8_t) (accelDiv & 0xFF) );

  /* Calculate the actual sample rate from the divider value */
  accelSampleRate = 1125.0 / (accelDiv + 1);

  return accelSampleRate;
}


void ICM_20948_wakeOnMotionITEnable(uint8_t womThreshold, float sampleRate)
{
	/* Make sure that the chip is not in sleep */
    ICM_20948_sleepModeEnable(false);

	/* And in continuous mode */
    ICM_20948_cycleModeEnable(false);

	/* Enable only the accelerometer */
	ICM_20948_sensorEnable(true, false, false);

	/* Set sample rate */
	ICM_20948_sampleRateSet(sampleRate);

	/* Set the bandwidth to 1210Hz */
	ICM_20948_accelBandwidthSet(ICM_20948_ACCEL_BW_1210HZ);

	/* Accel: 2G full scale */
	ICM_20948_accelFullscaleSet(ICM_20948_ACCEL_FULLSCALE_2G);

	/* Enable the Wake On Motion interrupt */
	ICM_20948_interruptEnable(false, true);
	nrf_delay_ms(50);

	/* Enable Wake On Motion feature */
	ICM_20948_registerWrite(ICM_20948_REG_ACCEL_INTEL_CTRL, ICM_20948_BIT_ACCEL_INTEL_EN | ICM_20948_BIT_ACCEL_INTEL_MODE);

	/* Set the wake on motion threshold value */
	ICM_20948_registerWrite(ICM_20948_REG_ACCEL_WOM_THR, womThreshold);

	/* Enable low power mode */
	ICM_20948_lowPowerModeEnter(true, false, false);

	// Keep track of WoM state
	imu_set_in_wom(true);
}


// void apply_stored_offsets(void)
// {
// 	uint8_t sensor_bias[84];
// 	int32_t acc_bias_q16[6] = {0}, gyro_bias_q16[6] = {0};
// 	uint8_t i, idx = 0;
// 	int rc;
	
	
// 	// TODO own implementation needed
// 	/* Retrieve Sel-test offsets stored in NV memory */
// //	if(flash_manager_readData(sensor_bias) != 0) {
// //		INV_MSG(INV_MSG_LEVEL_WARNING, "No bias values retrieved from NV memory !");
// //		return;
// //	}
	
// 	for(i = 0; i < 6; i++)
// 		gyro_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[i * sizeof(uint32_t)]));
// 	idx += sizeof(gyro_bias_q16);
// 	rc = inv_device_set_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,
// 		VSENSOR_CONFIG_TYPE_OFFSET, gyro_bias_q16, sizeof(gyro_bias_q16));
// 	check_rc(rc);
	
// 	for(i = 0; i < 6; i++)
// 		acc_bias_q16[i] = inv_dc_little8_to_int32((const uint8_t *)(&sensor_bias[idx + i * sizeof(uint32_t)]));
// 	idx += sizeof(acc_bias_q16);
// 	rc = inv_device_set_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER,
// 		VSENSOR_CONFIG_TYPE_OFFSET, acc_bias_q16, sizeof(acc_bias_q16));

// }



// void store_offsets(void)
// {
// 	int rc = 0;
// 	uint8_t i, idx = 0;
// 	int gyro_bias_q16[6] = {0}, acc_bias_q16[6] = {0};

// 	uint8_t sensor_bias[84] = {0};
	
// 	/* Strore Self-test bias in NV memory */
// 	rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_GYROSCOPE,
// 			VSENSOR_CONFIG_TYPE_OFFSET, gyro_bias_q16, sizeof(gyro_bias_q16));
// 	check_rc(rc);
// 	for(i = 0; i < 6; i++)
// 		inv_dc_int32_to_little8(gyro_bias_q16[i], &sensor_bias[i * sizeof(uint32_t)]);
// 	idx += sizeof(gyro_bias_q16);
	
// 	rc = inv_device_get_sensor_config(device, INV_SENSOR_TYPE_ACCELEROMETER,
// 			VSENSOR_CONFIG_TYPE_OFFSET, acc_bias_q16, sizeof(acc_bias_q16));
// 	check_rc(rc);
// 	for(i = 0; i < 6; i++)
// 		inv_dc_int32_to_little8(acc_bias_q16[i], &sensor_bias[idx + i * sizeof(uint32_t)]);
// 	idx += sizeof(acc_bias_q16);

// 	// TODO own implementation needed to store sensor_bias in non volatile memory
// 	// flash_manager_writeData(sensor_bias);
// }