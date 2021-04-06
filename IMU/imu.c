#include "imu.h"

/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively �Software�) is subject
 * to InvenSense and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws.
 *
 * InvenSense and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from InvenSense is strictly prohibited.
 *
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, THE SOFTWARE IS
 * PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED
 * TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * EXCEPT AS OTHERWISE PROVIDED IN A LICENSE AGREEMENT BETWEEN THE PARTIES, IN NO EVENT SHALL
 * INVENSENSE BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, OR ANY
 * DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * ________________________________________________________________________________________________________
 */
#include "../Devices/SerifHal.h"
#include "../Devices/HostSerif.h"
// include to low level system driver
// #include "MyTarget/SPI.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "DataConverter.h"

#include "usr_twi.h"

#include "nrf_drv_gpiote.h"

// Application scheduler
#include "app_scheduler.h"


// IMU
////////////////
//  INCLUDES  //
////////////////
#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "../Devices/SerifHal.h"
#include "../Devices/DeviceIcm20948.h"
#include "../DynamicProtocol/DynProtocol.h"
#include "../DynamicProtocol/DynProtocolTransportUart.h"
#include "../EmbUtils/Message.h"


// IMU params
#include "imu_params.h"

// Ringbuffer
#include "nrf_ringbuf.h"
// Create the ringbuffer for storing IMU data that has to be transmitted
NRF_RINGBUF_DEF(m_ringbuf, 512);


extern IMU imu;


/* Define msg level */
#define MSG_LEVEL INV_MSG_LEVEL_DEBUG

uint32_t bytes_available = 0;


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

//uint32_t imu_init(void);
static void check_rc(int rc);

uint32_t evt_scheduled = 0;
void gpiote_evt_sceduled(void * p_event_data, uint16_t event_size);
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

// Exported instance of SerifHal object:
// A pointer to this structure shall be passed to the Device API,
// for the device driver to access to the SPI/I2C bus.
// The device will not modify the object, so it can be declared const
// The underlying HW serial interface must be up and running before calling any
// device methods
//const inv_serif_hal_t my_serif_instance = {
//        my_serif_open_read_reg,  /* callback to read_reg low level method */
//        my_serif_open_write_reg, /* callback to read_reg low level method */
//        256,                     /* maximum number of bytes allowed per read transaction,
//                                    (limitation can come from internal buffer in the system driver) */
//        256,                     /* maximum number of bytes allowed per write transaction,
//                                    (limitation can come from internal buffer in the system driver) */
//        INV_SERIF_HAL_TYPE_I2C,  /* type of the serial interface (between SPI or I2C) */
//        (void *)0xDEAD           /* some context pointer passed to read_reg/write_reg callbacks */
//};

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
//				ret_code_t err = twi_open();
//				if(err == NRF_SUCCESS)
//				{
//					return 0;
//				}else{
//					return -1;
//				}
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
        // MyTarget_SPI_do_read_reg(&reg, 1, rbuffer, rlen);
	
				ret_code_t error = i2c_read_bytes( &m_twi, ICM_20948_I2C_ADDRESS, reg, rbuffer, rlen);
				if(error == NRF_SUCCESS)
				{
					return 0;
				}else{
					return -1;	// shall return a negative value on error
				}
					
//					if(twi_tx_done) 
//					{
//						return 0;
//					}
//					return -1;
	
}

int my_serif_open_write_reg(uint8_t reg, const uint8_t * wbuffer, uint32_t wlen)
{
        (void)reg, (void)wbuffer, (void)wlen;
        // MyTarget_SPI_do_write_reg(&reg, 1, wbuffer, wlen);
	
//				for( int i=0; i<wlen; i++)
//				{
					ret_code_t error = i2c_write_byte( &m_twi, ICM_20948_I2C_ADDRESS, reg, wbuffer, wlen, false);
//					i++;
//				}
				// TODO: return value is now always 0
	
				if(error == NRF_SUCCESS)
				{
					return 0;
				}else{
					return -1;	// shall return a negative value on error
				}
//					if(twi_tx_done) 
//					{
//						return 0;
//					}
//					return -1;
	
//        return 0; // shall return a negative value on error
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
extern uint64_t inv_icm20948_get_dataready_interrupt_time_us(void);

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

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern uint32_t nus_printf_custom(char* p_char);
extern uint32_t nus_send(void);


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
	
				//NRF_LOG_INFO("Sensor event!");
				//NRF_LOG_FLUSH();	
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
		

		
		uint32_t err_code;
		size_t len_in;
		
		uint8_t config_data[1];
		
		counterr++;
		
nrf_gpio_pin_set(19);

		
		
//			// Convert data to string over ble nus
//		sprintf(stringsend, "%d	- %f	%f	%f	%f - %f	%f	%f - %f	%f	%f - %f	%f	%f \n",
////					(inv_sensor_str(event->sensor)),
////					inv_icm20948_get_dataready_interrupt_time_us(),
////					(event->timestamp),
//					(counterr),
////					(ts_timestamp_get_ticks_u64(0)),
////					NRF_RTC1->COUNTER,
//					(event->data.quaternion.quat[0]),
//					(event->data.quaternion.quat[1]),
//					(event->data.quaternion.quat[2]),
//					(event->data.quaternion.quat[3]),
////					(event->data.quaternion.accuracy_flag),
////					(event->data.quaternion.accuracy),
//					
//					(event->data.gyr.vect[0]),
//					(event->data.gyr.vect[1]),
//					(event->data.gyr.vect[2]),
//					
//					(event->data.acc.vect[0]),
//					(event->data.acc.vect[1]),
//					(event->data.acc.vect[2]),
//					
//					(event->data.mag.vect[0]),
//					(event->data.mag.vect[1]),
//					(event->data.mag.vect[2])
//					
////					(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),
////					(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data)
////					
//					);
					

					

//	nus_printf_custom(stringsend);	

		switch(INV_SENSOR_ID_TO_TYPE(event->sensor)) {
		case INV_SENSOR_TYPE_RAW_ACCELEROMETER:
		case INV_SENSOR_TYPE_RAW_GYROSCOPE:
//			NRF_LOG_INFO("data event: %d %d %d %d", //inv_sensor_str(event->sensor),
//					(int)event->timestamp,
//					(int)event->data.raw3d.vect[0],
//					(int)event->data.raw3d.vect[1],
//					(int)event->data.raw3d.vect[2]);
				NRF_LOG_INFO("Data");
//					NRF_LOG_INFO("%d	%d	%d", event->data.raw3d.vect[0], event->data.raw3d.vect[1], event->data.raw3d.vect[2]);
			break;
		case INV_SENSOR_TYPE_ACCELEROMETER:
		case INV_SENSOR_TYPE_LINEAR_ACCELERATION:
		case INV_SENSOR_TYPE_GRAVITY:
			NRF_LOG_INFO("data event %s (mg): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.acc.vect[0]*1000),
					(int)(event->data.acc.vect[1]*1000),
					(int)(event->data.acc.vect[2]*1000),
					(int)(event->data.acc.accuracy_flag));
					
				// Put data in the ringbuffer
				len_in = sizeof(event->data.acc.vect);					
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(event->data.acc.vect), &len_in);
				APP_ERROR_CHECK(err_code);
					
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			NRF_LOG_INFO("data event %s (mdps): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.accuracy_flag));
	
					
				// Put data in the ringbuffer
				config_data[0] = ENABLE_GYRO;
				len_in = sizeof(config_data);
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, config_data, &len_in);
				len_in = sizeof(event->data.gyr.vect);					
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(event->data.gyr.vect), &len_in); //(uint8_t *)(event->data.quaternion.quat)
				APP_ERROR_CHECK(err_code);
					
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			NRF_LOG_INFO("data event %s (nT): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.accuracy_flag));
					
				// Put data in the ringbuffer
				len_in = sizeof(event->data.mag.vect);					
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(event->data.mag.vect), &len_in); //(uint8_t *)(event->data.quaternion.quat)
				APP_ERROR_CHECK(err_code);
					
			break;
		case INV_SENSOR_TYPE_UNCAL_GYROSCOPE:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (mdps): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.bias[0]*1000),
					(int)(event->data.gyr.bias[1]*1000),
					(int)(event->data.gyr.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_UNCAL_MAGNETOMETER:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s (nT): %d %d %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.bias[0]*1000),
					(int)(event->data.mag.bias[1]*1000),
					(int)(event->data.mag.bias[2]*1000));
			break;
		case INV_SENSOR_TYPE_GAME_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_ROTATION_VECTOR:
		case INV_SENSOR_TYPE_GEOMAG_ROTATION_VECTOR:
		{
				// New IMU data is available: count how many bytes are available
				bytes_available++;
//			NRF_LOG_INFO("Bytes: %d", bytes_available);
			
//			NRF_LOG_INFO("%s:	%d %d %d %d", inv_sensor_str(event->sensor),
//					(int)(event->data.quaternion.quat[0]*1000),
//					(int)(event->data.quaternion.quat[1]*1000),
//					(int)(event->data.quaternion.quat[2]*1000),
//					(int)(event->data.quaternion.quat[3]*1000));
//					(int)(event->data.gyr.accuracy_flag),
//					(int)(event->data.acc.accuracy_flag),	
//					(int)(event->data.mag.accuracy_flag), // 0 - 3: not calibrated - fully calibrated
//					(int)(event->data.quaternion.accuracy_flag));
				
				// Put data in the ringbuffer
				config_data[0] = ENABLE_QUAT6;
				len_in = sizeof(config_data);
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, config_data, &len_in);
				len_in = sizeof(event->data.quaternion.quat);	
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(event->data.quaternion.quat), &len_in); //(uint8_t *)(event->data.quaternion.quat)
				APP_ERROR_CHECK(err_code);

				break;
			}
		case INV_SENSOR_TYPE_ORIENTATION:
//			NRF_LOG_INFO("data event %s (e-3):, %d, %d, %d, Accuracy: %d ", inv_sensor_str(event->sensor),
//					(int)(event->data.orientation.x*1000),
//					(int)(event->data.orientation.y*1000),
//					(int)(event->data.orientation.z*1000),
//					(int)(event->data.orientation.accuracy_flag*1000)); // 0 - 3: not calibrated - fully calibrated
		NRF_LOG_INFO("%d, %d, %d, %d, %d, %d", // rewritten write funtion to allow easier plotting
					(int)(event->data.orientation.x),
					(int)(event->data.orientation.y),
					(int)(event->data.orientation.z),
//					(int)(event->data.orientation.accuracy_flag),
					(int)(event->data.gyr.accuracy_flag),
					(int)(event->data.acc.accuracy_flag),	
					(int)(event->data.mag.accuracy_flag)); // 0 - 3: not calibrated - fully calibrated
					
					float buffer_orientation[3];
					buffer_orientation[0] = event->data.orientation.x;
					buffer_orientation[1] = event->data.orientation.y;
					buffer_orientation[2] = event->data.orientation.z;
					
				// Put data in the ringbuffer
				len_in = sizeof(buffer_orientation);					
				err_code = nrf_ringbuf_cpy_put(&m_ringbuf, (uint8_t *)(buffer_orientation), &len_in); //(uint8_t *)(event->data.quaternion.quat)
				APP_ERROR_CHECK(err_code);
					
			break;
		case INV_SENSOR_TYPE_BAC:
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
		case INV_SENSOR_TYPE_STEP_DETECTOR:
		case INV_SENSOR_TYPE_SMD:
		case INV_SENSOR_TYPE_B2S:
		case INV_SENSOR_TYPE_TILT_DETECTOR:
		default:
			INV_MSG(INV_MSG_LEVEL_INFO, "data event %s : ...", inv_sensor_str(event->sensor));
			break;
	}
		
	nrf_gpio_pin_clear(19);
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




/* Interrupt pin handeler callback function */
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
		nrf_gpio_pin_toggle(25);
	
		uint32_t err_code;
	
		if(pin == INT_PIN)
		{
				// If there are already events in the queue
				if(evt_scheduled > 0)
				{
					evt_scheduled++;
				}
				// If there are not yet any events in the queue, schedule event. In gpiote_evt_sceduled all callbacks are called
				else
				{
					evt_scheduled++;
					err_code = app_sched_event_put(0, 0, gpiote_evt_sceduled);
					APP_ERROR_CHECK(err_code);
				}
		}
		
	nrf_gpio_pin_toggle(25);
}



// Event handler DIY
/**@brief GPIOTE sceduled handler, executed in main-context.
 */
void gpiote_evt_sceduled(void * p_event_data, uint16_t event_size)
{
    while ( (evt_scheduled > 0) )//&& m_mpu9250.enabled) TODO check when IMU is enabled or not
    {
				nrf_gpio_pin_set(20);
			// Poll all data from IMU
				inv_device_poll(device);
				nrf_gpio_pin_clear(20);
				evt_scheduled--;
    }
}

static uint16_t packet_length_temp;
void set_imu_packet_length()
{
	packet_length_temp = 0;
	imu.packet_length = 0;
	
	
	// Data length is length of 3 or 4 floats + 1 byte for data classification
	if(imu.gyro_enabled)
	{
		packet_length_temp += (3 * sizeof(float)) + 1;
		imu.packet_length += (3 * sizeof(float)) + 1;
	}
	if(imu.accel_enabled)
	{
		packet_length_temp += (3 * sizeof(float)) + 1;
		imu.packet_length += (3 * sizeof(float)) + 1;
	}
	if(imu.mag_enabled)
	{
		packet_length_temp += (3 * sizeof(float)) + 1;
		imu.packet_length += (3 * sizeof(float)) + 1;
	}
	if(imu.quat6_enabled)
	{
		packet_length_temp += (4 * sizeof(float)) + 1;
		imu.packet_length += (4 * sizeof(float)) + 1;
	}
	if(imu.quat9_enabled)
	{
		packet_length_temp += (4 * sizeof(float)) + 1;
		imu.packet_length += (4 * sizeof(float)) + 1;
	}
	if(imu.euler_enabled)
	{
		packet_length_temp += (3 * sizeof(float)) + 1;
		imu.packet_length += (3 * sizeof(float)) + 1;
	}
	
	NRF_LOG_INFO("Packet Len set to: %d", imu.packet_length);
}

uint32_t imu_enable_sensors(IMU imu)
{
		int rc = 0;
	
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
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR);
		check_rc(rc);
		}
		// If enabled: Start 9DoF quaternion output
		if(imu.quat9_enabled)
		{
		NRF_LOG_INFO("Start QUAT9");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ROTATION_VECTOR, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ROTATION_VECTOR);
		check_rc(rc);
		}
		// If enabled: Start Euler angles
		if(imu.euler_enabled)
		{
		NRF_LOG_INFO("Start 9DoF EULER");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ORIENTATION, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ORIENTATION);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Gyroscope output
		if(imu.gyro_enabled)
		{
		NRF_LOG_INFO("Start GYRO");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_GYROSCOPE, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_GYROSCOPE);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Accelerometer output
		if(imu.accel_enabled)
		{
		NRF_LOG_INFO("Start ACCEL");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_ACCELEROMETER, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_ACCELEROMETER);
		check_rc(rc);
		}
		// If enabled: Start Calibrated Accelerometer output
		if(imu.mag_enabled)
		{
		NRF_LOG_INFO("Start MAG");
		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		rc += inv_device_set_sensor_period(device, INV_SENSOR_TYPE_MAGNETOMETER, imu.period);
		check_rc(rc);
		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_MAGNETOMETER);
		check_rc(rc);
		}
		
		/* Activity classification */
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_BAC);
//		check_rc(rc);
		
		/* Step Counter */
//		rc += inv_device_ping_sensor(device, INV_SENSOR_TYPE_STEP_COUNTER);
//		check_rc(rc);
//		rc += inv_device_start_sensor(device, INV_SENSOR_TYPE_STEP_COUNTER);
//		check_rc(rc);
		
//		inv_device_set_sensor_timeout(device, INV_SENSOR_TYPE_GAME_ROTATION_VECTOR, 5);
		
			return NRF_SUCCESS;
}


uint32_t imu_init(void)
{
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
		
		rc = inv_host_serif_open(&my_serif_instance);
		check_rc(rc);
		
		NRF_LOG_INFO("i2c init");
		NRF_LOG_FLUSH();
		
		NRF_LOG_INFO("icm20948 init");
		NRF_LOG_FLUSH();
		/*
		 * Create ICM20948 Device 
		 * Pass to the driver:
		 * - reference to serial interface object,
		 * - reference to listener that will catch sensor events,
		 */
		inv_device_icm20948_init(&device_icm20948, &my_serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
//		inv_device_icm20948_init2(&device_icm20948, &my_serif_instance, &sensor_listener, dmp3_image, sizeof(dmp3_image));
		NRF_LOG_FLUSH();
		
		NRF_LOG_INFO("icm20948 get base");
		NRF_LOG_FLUSH();
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
		
		nrf_delay_ms(500);
		
		/* Configure and initialize the Icm20948 device */
		NRF_LOG_INFO("Setting up ICM20948");
		NRF_LOG_FLUSH();
		rc += inv_device_setup(device);
		check_rc(rc);
		
		// Load DMP
		NRF_LOG_INFO("Load DMP Image");
		NRF_LOG_FLUSH();
		rc += inv_device_load(device, NULL, dmp3_image, sizeof(dmp3_image), true /* verify */, NULL);
		check_rc(rc);

		imu_enable_sensors(imu);

		return NRF_SUCCESS;
}

uint32_t imu_get_bytes_available(void)
{
	return bytes_available;
}

void imu_set_bytes_available(uint32_t bytes)
{
	bytes_available = bytes;
}

size_t get_imu_packet_length(IMU imu)
{
	return imu.packet_length;
}

void IMU_data_get(uint8_t * data, uint16_t * len)
{
		uint32_t err_code;

		*len = imu.packet_length;
	
		NRF_LOG_INFO("len_out: %d   %d", *len, packet_length_temp);
	
		uint8_t temp[*len];
		err_code = nrf_ringbuf_cpy_get(&m_ringbuf, temp, (size_t *) len);
		APP_ERROR_CHECK(err_code);
	
	NRF_LOG_INFO("temp: %X %X %X %X", temp[0], temp[1], temp[2], temp[3]);
//		NRF_LOG_INFO("%d %d %d %d", (int)(quat[0]*1000),(int)(quat[1]*1000),(int)(quat[2]*1000),(int)(quat[3]*1000));
//		NRF_LOG_INFO("%d %d %d", (int)(temp[0]*1000),(int)(temp[1]*1000),(int)(temp[2]*1000));
		memcpy(data, temp, *len);
}

void usr_ringbuf_init(void)
{
	nrf_ringbuf_init(&m_ringbuf);
}

