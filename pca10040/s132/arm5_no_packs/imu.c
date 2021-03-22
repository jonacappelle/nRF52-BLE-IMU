#include "imu.h"

/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2015-2015 InvenSense Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
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
#include "Invn/Devices/SerifHal.h"
#include "Invn/Devices/HostSerif.h"
// include to low level system driver
// #include "MyTarget/SPI.h"
#include "nrf_drv_twi.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "DataConverter.h"

#include "usr_twi.h"

extern const nrf_drv_twi_t m_twi;

extern ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);
extern ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop);
extern void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
#define ICM_20948_I2C_ADDRESS		0x69U


// forward declarations
int my_serif_open_adapter(void);
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
        0,
        my_serif_open_read_reg,
        my_serif_open_write_reg,
				my_adapter_register_interrupt_callback,
        256,
				256,
        INV_HOST_SERIF_TYPE_I2C
};

int my_serif_open_adapter(void)
{
				twi_init();
				return 0;
}

int my_adapter_register_interrupt_callback(void (*interrupt_cb)(void * context, int int_num), void * context)
{
//				(*interrupt_cb) = twi_handler;
//	
//				
//				context = 0;
			
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
//				NRF_LOG_INFO("us value requested");
	
        (void)us;

	
				nrf_delay_us(us);
}




/////////// SENSOR EVENTS \\\\\\\\\\\\\\\\\\\\\\

#include "Invn/Devices/SerifHal.h"
#include "Invn/Devices/DeviceIcm20948.h"
#include "Invn/DynamicProtocol/DynProtocol.h"
#include "Invn/DynamicProtocol/DynProtocolTransportUart.h"
// INV_MSG functionality
#include "Invn/EmbUtils/Message.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

extern uint32_t nus_printf_custom(char* p_char);
extern uint32_t nus_send();


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
	
	
	/* Send data from IMU to central */
	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	
	


	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	if(event->status == INV_SENSOR_STATUS_DATA_UPDATED) {
		
		counterr++;
		
		float dummy_data = 1.001;
		
		

		
		
			// Convert data to string over ble nus
		sprintf(stringsend, "%d	- %f	%f	%f	%f - %f	%f	%f - %f	%f	%f - %f	%f	%f \n",
//					(inv_sensor_str(event->sensor)),
//					inv_icm20948_get_dataready_interrupt_time_us(),
//					(event->timestamp),
					(counterr),
//					(ts_timestamp_get_ticks_u64(0)),
//					NRF_RTC1->COUNTER,
					(event->data.quaternion.quat[0]),
					(event->data.quaternion.quat[1]),
					(event->data.quaternion.quat[2]),
					(event->data.quaternion.quat[3]),
//					(event->data.quaternion.accuracy_flag),
//					(event->data.quaternion.accuracy),
					
					(event->data.gyr.vect[0]),
					(event->data.gyr.vect[1]),
					(event->data.gyr.vect[2]),
					
					(event->data.acc.vect[0]),
					(event->data.acc.vect[1]),
					(event->data.acc.vect[2]),
					
					(event->data.mag.vect[0]),
					(event->data.mag.vect[1]),
					(event->data.mag.vect[2])
					
//					(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),
//					(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data),(dummy_data)
//					
					);
					

					

//	nus_printf_custom(stringsend);
//	nus_send();

	
//	for(int i=0; i<132; i++)
//	{
//	test_troughput_array[i] = i;
//	}
//	nus_send(&test_troughput_array, 132);
	
//	NRF_LOG_INFO("Counter: %d", counterr);
	
	
	

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
			break;
		case INV_SENSOR_TYPE_GYROSCOPE:
			NRF_LOG_INFO("data event %s (mdps): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.gyr.vect[0]*1000),
					(int)(event->data.gyr.vect[1]*1000),
					(int)(event->data.gyr.vect[2]*1000),
					(int)(event->data.gyr.accuracy_flag));
			break;
		case INV_SENSOR_TYPE_MAGNETOMETER:
			NRF_LOG_INFO("data event %s (nT): %d %d %d %d", inv_sensor_str(event->sensor),
					(int)(event->data.mag.vect[0]*1000),
					(int)(event->data.mag.vect[1]*1000),
					(int)(event->data.mag.vect[2]*1000),
					(int)(event->data.mag.accuracy_flag));
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
//			NRF_LOG_INFO("RV: Accuracy: %d %d %d %d", //inv_sensor_str(event->sensor),
////					(int)(event->data.quaternion.quat[0]*1000),
////					(int)(event->data.quaternion.quat[1]*1000),
////					(int)(event->data.quaternion.quat[2]*1000),
////					(int)(event->data.quaternion.quat[3]*1000),
//					(int)(event->data.gyr.accuracy_flag),
//					(int)(event->data.acc.accuracy_flag),	
//					(int)(event->data.mag.accuracy_flag), // 0 - 3: not calibrated - fully calibrated
//					(int)(event->data.quaternion.accuracy_flag));
			break;
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
//	NRF_LOG_INFO("Message printe ENABLED!!");
	NRF_LOG_FLUSH();
//	char str_length[100];
//	sprintf(str_length, "%s", str);
}






