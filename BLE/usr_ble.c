#include "usr_ble.h"



#include <stdint.h>
#include <string.h>
#include "nordic_common.h"
#include "nrf.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "app_timer.h"
#include "ble_nus.h"
#include "app_uart.h"
#include "app_util_platform.h"
#include "bsp_btn_ble.h"
#include "nrf_pwr_mgmt.h"

#if defined (UART_PRESENT)
#include "nrf_uart.h"
#endif
#if defined (UARTE_PRESENT)
#include "nrf_uarte.h"
#endif

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

///////////////////////////////////////////////

#include "nrf_delay.h"

// TimeSync
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

// IMU params
#include "imu_params.h"
#include "imu.h"
extern IMU imu;
extern imu_data_t imu_data;

// Application scheduler
#include "app_scheduler.h"

// Add TMS service for transmitting IMU data
#include "ble_tms.h"

// BLE Battery service
#include "ble_bas.h"

#include "usr_util.h"


int countrrr = 0;
bool nus_buffer_full = false;

#define FREQ_TO_MS(x) ((1.000/x)*1000)

uint32_t led_blink_tick = 0;



///////////////////////////////////////////////
#define SYNC_FREQ	2 // Hz

// In increments of 2.5 ms
uint32_t sync_interval_int_time = 4; // Default 100 Hz transmission rate

static bool m_gpio_trigger_enabled = 0;
static bool m_imu_trigger_enabled = 0;


///////////////////////////////////////////////

#define APP_BLE_CONN_CFG_TAG            1                                           /**< A tag identifying the SoftDevice BLE configuration. */

#define DEVICE_NAME                     "IMU2"                               /**< Name of device. Will be included in the advertising data. */
#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_VENDOR_BEGIN                  /**< UUID type for the Nordic UART Service (vendor specific). */

#define APP_BLE_OBSERVER_PRIO           3                                           /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define APP_ADV_INTERVAL                150//64                                          /**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#define APP_ADV_DURATION                18000                                       /**< The advertising duration (180 seconds) in units of 10 milliseconds. */

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS(10, UNIT_1_25_MS)             /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(75, UNIT_1_25_MS)             /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
#define SLAVE_LATENCY                   0                                           /**< Slave latency. */
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)             /**< Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units. */
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)                       /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000)                      /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                           /**< Number of attempts before giving up the connection parameter negotiation. */

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define UART_TX_BUF_SIZE                1024                                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE                1024                                         /**< UART RX buffer size. */


BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static uint16_t   m_ble_nus_max_data_len = 247 - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */

#define TMS_SERVICE_UUID_TYPE       BLE_UUID_TYPE_VENDOR_BEGIN

static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
    {BLE_UUID_TMS_SERVICE, TMS_SERVICE_UUID_TYPE}, // Added for TMS service
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}
};


//////////////////
// TMS Motion service
//////////////////

static void ble_tms_evt_handler(ble_tms_t        * p_tms,
                                ble_tms_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length)
{
    NRF_LOG_DEBUG("ble_tms_evt_handler called");
    uint32_t err_code;
    
    switch (evt_type)
    {
        case BLE_TMS_EVT_NOTIF_TAP:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_TAP - %d\r\n", p_tms->is_tap_notif_enabled);
            // if (p_tms->is_tap_notif_enabled)
            // {
            //     err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_TAP);
            //     APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_TAP);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_TMS_EVT_NOTIF_ADC:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_ADC - %d\r\n", p_tms->is_adc_notif_enabled);
            // if (p_tms->is_orientation_notif_enabled)
            // {
            //     imu.quat6_enabled = 1;
                
            //     // err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_ORIENTATION);
            //     // APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     imu.quat6_enabled = 0;

            //     // err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_ORIENTATION);
            //     // APP_ERROR_CHECK(err_code);
            // }

            // // Pass change IMU settings to event handler
            // err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            // APP_ERROR_CHECK(err_code);
            break;

        case BLE_TMS_EVT_NOTIF_QUAT:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_QUAT - %d\r\n", p_tms->is_quat_notif_enabled);
            // if (p_tms->is_quat_notif_enabled)
            // {
            //     imu.quat6_enabled = 1;
                
            //     // err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_QUAT);
            //     // APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     imu.quat6_enabled = 0;

            //     // err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_QUAT);
            //     // APP_ERROR_CHECK(err_code);
            // }

            // // Pass change IMU settings to event handler
            // err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            // APP_ERROR_CHECK(err_code);

            break;

        case BLE_TMS_EVT_NOTIF_PEDOMETER:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_PEDOMETER - %d\r\n", p_tms->is_pedo_notif_enabled);
            // if (p_tms->is_pedo_notif_enabled)
            // {
            //     err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_PEDOMETER);
            //     APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_PEDOMETER);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_TMS_EVT_NOTIF_RAW:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_RAW - %d\r\n", p_tms->is_raw_notif_enabled);
            // if (p_tms->is_raw_notif_enabled)
            // {
            //     imu.gyro_enabled = 1;
            //     imu.accel_enabled = 1;
            //     imu.mag_enabled = 1;
            //     // err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_RAW);
            //     // APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     imu.gyro_enabled = 0;
            //     imu.accel_enabled = 0;
            //     imu.mag_enabled = 0;
            //     // err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_RAW);
            //     // APP_ERROR_CHECK(err_code);
            // }

            // // Pass change IMU settings to event handler
            // err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            // APP_ERROR_CHECK(err_code);

            break;

        case BLE_TMS_EVT_NOTIF_EULER:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_EULER - %d\r\n", p_tms->is_euler_notif_enabled);
            // if (p_tms->is_euler_notif_enabled)
            // {
            //     imu.euler_enabled = 1;
            //     // err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_EULER);
            //     // APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     imu.euler_enabled = 0;
            //     // err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_EULER);
            //     // APP_ERROR_CHECK(err_code);
            // }

            // // Pass change IMU settings to event handler
            // err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            // APP_ERROR_CHECK(err_code);

            break;

        case BLE_TMS_EVT_NOTIF_ROT_MAT:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_ROT_MAT - %d\r\n", p_tms->is_rot_mat_notif_enabled);
            // if (p_tms->is_rot_mat_notif_enabled)
            // {
            //     err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_ROT_MAT);
            //     APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_ROT_MAT);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_TMS_EVT_NOTIF_HEADING:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_HEADING - %d\r\n", p_tms->is_heading_notif_enabled);
            // if (p_tms->is_heading_notif_enabled)
            // {
            //     err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_HEADING);
            //     APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_HEADING);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_TMS_EVT_NOTIF_GRAVITY:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_GRAVITY - %d\r\n", p_tms->is_gravity_notif_enabled);
            // if (p_tms->is_gravity_notif_enabled)
            // {
            //     err_code = drv_motion_enable(DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR);
            //     APP_ERROR_CHECK(err_code);
            // }
            // else
            // {
            //     err_code = drv_motion_disable(DRV_MOTION_FEATURE_MASK_GRAVITY_VECTOR);
            //     APP_ERROR_CHECK(err_code);
            // }
            break;

        case BLE_TMS_EVT_CONFIG_RECEIVED:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_CONFIG_RECEIVED - %d\r\n", length);
            APP_ERROR_CHECK_BOOL(length == sizeof(ble_tms_config_t));

            ble_tms_config_t received_config;
            memcpy(&received_config, p_data, length);

            NRF_LOG_INFO("FREQUENCY %d Hz", received_config.motion_freq_hz);

            imu.gyro_enabled = received_config.gyro_enabled;
            imu.accel_enabled = received_config.accel_enabled;
            imu.mag_enabled = received_config.mag_enabled;
            imu.quat6_enabled = received_config.quat6_enabled;
            imu.quat9_enabled = received_config.quat9_enabled;
            imu.euler_enabled = received_config.euler_enabled;
            if(received_config.motion_freq_hz != 0) imu.period = FREQ_TO_MS(received_config.motion_freq_hz);
            imu.sync = received_config.sync_enabled;
            imu.stop = received_config.stop;
            imu.adc = received_config.adc_enabled;
            imu.sync_start_time = received_config.sync_start_time;

            // Stop syncing
            m_imu_trigger_enabled = imu.sync;

            NRF_LOG_INFO("received_config.motion_freq_hz %d", received_config.motion_freq_hz);
            NRF_LOG_INFO("imu.period %d", imu.period);
            NRF_LOG_INFO("ADC received config: %d", received_config.adc_enabled);
            NRF_LOG_INFO("ADC: %d", imu.adc);

            // Pass change IMU settings to event handler
            err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            APP_ERROR_CHECK(err_code);

            break;

        default:
            break;
    }

}

ble_tms_t              m_tms;

static ble_tms_config_t     * m_config;

uint32_t usr_tms_init(void)
{
    uint32_t err_code;
    ble_tms_init_t        tms_init;

    memset(&tms_init, 0, sizeof(tms_init));

    // tms_init.p_init_config = m_config;

    // Init with all parameters to default value
    tms_init.p_init_config->gyro_enabled = 0;
    tms_init.p_init_config->accel_enabled = 0;
    tms_init.p_init_config->mag_enabled = 0;
    tms_init.p_init_config->euler_enabled = 0;
    tms_init.p_init_config->quat6_enabled = 0;
    tms_init.p_init_config->quat9_enabled = 0;
    tms_init.p_init_config->wom_enabled = 1;
    tms_init.p_init_config->motion_freq_hz = 100;


    tms_init.evt_handler = ble_tms_evt_handler;

    NRF_LOG_INFO("motion_service_init: ble_tms_init \r\n");
    NRF_LOG_DEBUG("ble_tms_init");
    err_code = ble_tms_init(&m_tms, &tms_init);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_ERROR("FAILED - %d\r\n", err_code);
        return err_code;
    }
}

void tms_test(void)
{
    // Test TMS data transmission
    ble_tms_quat_t data;
    data.w = 1;
    data.x = 2;
    data.y = 3;
    data.z = 4;
    (void)ble_tms_quat_set(&m_tms, &data);
    // NRF_LOG_INFO("TMS Send!");
}





BLE_BAS_DEF(m_bas);                                                 /**< Structure used to identify the battery service. */

uint32_t usr_bas_init(void)
{
    uint32_t err_code;
    ble_bas_init_t     bas_init;
    
    // Initialize Battery Service.
    memset(&bas_init, 0, sizeof(bas_init));

    bas_init.evt_handler          = NULL;
    bas_init.support_notification = true;
    bas_init.p_report_ref         = NULL;
    bas_init.initial_batt_level   = 100;

    // Here the sec level for the Battery Service can be changed/increased.
    bas_init.bl_rd_sec        = SEC_OPEN;
    bas_init.bl_cccd_wr_sec   = SEC_OPEN;
    bas_init.bl_report_rd_sec = SEC_OPEN;

    err_code = ble_bas_init(&m_bas, &bas_init);
    APP_ERROR_CHECK(err_code);
}


void battery_level_update(uint8_t battery_level)
{
    ret_code_t err_code;

    err_code = ble_bas_battery_level_update(&m_bas, battery_level, BLE_CONN_HANDLE_ALL);
    if ((err_code != NRF_SUCCESS) &&
        (err_code != NRF_ERROR_INVALID_STATE) &&
        (err_code != NRF_ERROR_RESOURCES) &&
        (err_code != NRF_ERROR_BUSY) &&
        (err_code != BLE_ERROR_GATTS_SYS_ATTR_MISSING)
       )
    {
        APP_ERROR_HANDLER(err_code);
    }
}









/**@brief Function for handling Queued Write Module errors.
 *
 * @details A pointer to this function will be passed to each service which may need to inform the
 *          application about an error.
 *
 * @param[in]   nrf_error   Error code containing information about what went wrong.
 */
static void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the timer module.
 */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for the GAP initialization.
 *
 * @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
 *          the device. It also sets the permissions and appearance.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *) DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}


// Event handler DIY
/**@brief GPIOTE sceduled handler, executed in main-context.
 */
void imu_config_evt_sceduled(void * p_event_data, uint16_t event_size)
{
		// Enable sensor parameters based on received configuration
		uint32_t err_code;

        #if IMU_ENABLED == 1
		err_code =  imu_enable_sensors(imu);
		APP_ERROR_CHECK(err_code);
        #endif

        sync_interval_int_time = (imu.period / 2.5);        

        // if(m_imu_trigger_enabled) 
        // {
        //     NRF_LOG_INFO("if(m_imu_trigger_enabled) is true");
        //     ts_imu_trigger_enable();
        // }

    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;
    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec);
    NRF_LOG_INFO("Time_target (ticks) %d", time_target);

    NRF_LOG_INFO("ime.sync_start_time %d", imu.sync_start_time);

        if( ( imu.sync ) && ( imu.sync_start_time != 0) )
        {
        err_code = ts_set_trigger(imu.sync_start_time, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
        APP_ERROR_CHECK(err_code);
        m_imu_trigger_enabled = 1 ;
        }else{
            m_imu_trigger_enabled = 0;
        }

        // Notify TimeSync to start synchronizing again
        // ts_state_set(m_imu_trigger_enabled);

        NRF_LOG_INFO("sync_interval_int_time: %d", sync_interval_int_time);

        // TODO
        // adc_enable(imu);
}


/**@brief Function for handling the data from the Nordic UART Service.
 *
 * @details This function will process the data received from the Nordic UART BLE Service and send
 *          it to the UART module.
 *
 * @param[in] p_evt       Nordic UART Service event.
 */
/**@snippet [Handling the data received over BLE] */
static void nus_data_handler(ble_nus_evt_t * p_evt)
{
		uint32_t err_code;
	
    switch (p_evt->type)
    {
		
			case BLE_NUS_EVT_RX_DATA:
			{
						uint8_t len = p_evt->params.rx_data.length;
						uint8_t buffer[len];
				

						// Copy data into buffer
						memcpy(buffer, p_evt->params.rx_data.p_data, len);
				
					NRF_LOG_INFO("BLE_NUS_EVT_RX_DATA: %X %X len: %d", buffer[0], buffer[1], len);
				
				
						// Reset all params to 0 to start with
						imu.gyro_enabled = 0;
						imu.accel_enabled = 0;
						imu.mag_enabled = 0;
						imu.quat6_enabled = 0;
						imu.quat9_enabled = 0;
						imu.euler_enabled = 0;
			
						for(uint8_t i=0; i<len; i++)
						{
							uint8_t temp = buffer[i];
							
							// Set struct parameters based on what configuration is set at central
							switch (temp)
							{
									case ENABLE_QUAT6:
										imu.quat6_enabled = 1;
										NRF_LOG_INFO("ENABLE_QUAT6 Received");
										break;

									case ENABLE_QUAT9:
										imu.quat9_enabled = 1;
										NRF_LOG_INFO("ENABLE_QUAT9 Received");
										break;
									
									case ENABLE_EULER:
										imu.euler_enabled = 1;
										NRF_LOG_INFO("ENABLE_EULER Received");
										break;
									
									case ENABLE_GYRO:
										imu.gyro_enabled = 1;
										NRF_LOG_INFO("ENABLE_GYRO Received");
										break;
									
									case ENABLE_ACCEL:
										imu.accel_enabled = 1;
										NRF_LOG_INFO("ENABLE_ACCEL Received");
										break;
									
									case ENABLE_MAG:
										imu.mag_enabled = 1;
										NRF_LOG_INFO("ENABLE_MAG Received");
										break;
									
									case STOP_IMU:
										imu.stop = 1;
										NRF_LOG_INFO("STOP_IMU Received");
										break;
									
									default:
										imu.gyro_enabled = 0;
										imu.accel_enabled = 0;
										imu.mag_enabled = 0;
										imu.quat6_enabled = 0;
										imu.quat9_enabled = 0;
										imu.euler_enabled = 0;
										NRF_LOG_INFO("Unknown Config Received");
										break;
							}
						}
						
						set_imu_packet_length();
						
						// Pass change IMU settings to event handler
						err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
						APP_ERROR_CHECK(err_code);
						
						
//					NRF_LOG_DEBUG("Received data from BLE NUS. Writing data on UART.");
//					NRF_LOG_HEXDUMP_DEBUG(p_evt->params.rx_data.p_data, p_evt->params.rx_data.length);

//					for (uint32_t i = 0; i < p_evt->params.rx_data.length; i++)
//					{
//							do
//							{
//									err_code = app_uart_put(p_evt->params.rx_data.p_data[i]);
//									if ((err_code != NRF_SUCCESS) && (err_code != NRF_ERROR_BUSY))
//									{
//											NRF_LOG_ERROR("Failed receiving NUS message. Error 0x%x. ", err_code);
//											APP_ERROR_CHECK(err_code);
//									}
//							} while (err_code == NRF_ERROR_BUSY);
//					}
//					if (p_evt->params.rx_data.p_data[p_evt->params.rx_data.length - 1] == '\r')
//					{
//							while (app_uart_put('\n') == NRF_ERROR_BUSY);
//					}
					break;
				}
				// Added //
				// When BLE NUS
			case BLE_NUS_EVT_TX_RDY:
						nus_buffer_full = false;
//						NRF_LOG_INFO("BLE_NUS_EVT_TX_RDY");
					break;
            case BLE_NUS_EVT_COMM_STARTED:
                    NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STARTED");
                    break;
            case BLE_NUS_EVT_COMM_STOPPED:
                    NRF_LOG_DEBUG("BLE_NUS_EVT_COMM_STOPPED");
                    break;

			default:
					break;
			}
}
/**@snippet [Handling the data received over BLE] */


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling an event from the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module
 *          which are passed to the application.
 *
 * @note All this function does is to disconnect. This could have been done by simply setting
 *       the disconnect_on_fail config parameter, but instead we use the event handler
 *       mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t * p_evt)
{
    uint32_t err_code;

    if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}


/**@brief Function for handling errors from the Connection Parameters module.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}


/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;

    memset(&cp_init, 0, sizeof(cp_init));

    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;

    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for putting the chip into sleep mode.
 *
 * @note This function will not return.
 */
static void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_DEBUG("Fast Advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_DEBUG("Advertising idle");
            sleep_mode_enter();
            break;
        default:
            break;
    }
}


/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    uint32_t err_code;

    // Pass softdevice calls back to TMS handler
    ble_tms_on_ble_evt(&m_tms, p_ble_evt);

    switch (p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
            NRF_LOG_INFO("Connected");
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            // Stop IMU when BLE connection disconnects
            imu.gyro_enabled = 0;
            imu.accel_enabled = 0;
            imu.mag_enabled = 0;
            imu.euler_enabled = 0;
            imu.quat6_enabled = 0;
            imu.quat9_enabled = 0;

            // Pass change IMU settings to event handler
            err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        } break;

        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTC_EVT_TIMEOUT:
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        case BLE_GATTS_EVT_TIMEOUT:
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            // No implementation needed.
            break;
    }
}


/**@brief Function for the SoftDevice initialization.
 *
 * @details This function initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // CHANGES
    // Configure BLE buffer: store more packets than usable in one connection interval in BLE buffer.
    ble_cfg_t ble_cfg;

    memset(&ble_cfg, 0, sizeof ble_cfg);
    ble_cfg.conn_cfg.conn_cfg_tag                     = APP_BLE_CONN_CFG_TAG;
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 20;
    err_code = sd_ble_cfg_set(BLE_CONN_CFG_GATTS, &ble_cfg, ram_start);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("sd_ble_cfg_set err_code: %d", err_code);



    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
}


/**@brief Function for handling events from the GATT library. */
void gatt_evt_handler(nrf_ble_gatt_t * p_gatt, nrf_ble_gatt_evt_t const * p_evt)
{
    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
		
		// Added//
		if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        NRF_LOG_INFO("MTU size is set to 0x%X(%d)", p_evt->params.att_mtu_effective, p_evt->params.att_mtu_effective);
    }

    if ((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_DATA_LENGTH_UPDATED))
    {
         NRF_LOG_INFO("LL packet size is set to %d", p_gatt->data_length);
    }
}


/**@brief Function for initializing the GATT library. */
void gatt_init(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE); // 247 max size
    APP_ERROR_CHECK(err_code);
}



static void ts_gpio_trigger_enable(void)
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    uint32_t err_code;

    if (m_gpio_trigger_enabled)
    {
        return;
    }

    // Round up to nearest second to next 2000 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (2000 * 2);
    time_target = (time_target / 2000) * 2000;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_gpio_trigger_enabled = true;
}

static void ts_gpio_trigger_disable(void)
{
    m_gpio_trigger_enabled = false;
}


void ts_imu_trigger_enable(void)
{
    NRF_LOG_INFO("ts_imu_trigger_enable()");

    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    uint32_t err_code;

    if (m_imu_trigger_enabled)
    {
        return;
    }

    // Round up to nearest second to next 2000 ms to start toggling.
    // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.
    // If Sync packet is received within this timestamp - the peripherals will trigger at the same time.

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

    time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (2000 * 2);
    time_target = (time_target / 2000) * 2000;

    err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
    APP_ERROR_CHECK(err_code);

    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

    m_imu_trigger_enabled = true;
}

static void ts_imu_trigger_disable(void)
{
    NRF_LOG_INFO("ts_imu_trigger_disable()");
    m_imu_trigger_enabled = false;
}

static void ts_evt_callback(const ts_evt_t* evt)
{
    // NRF_LOG_INFO("ts_evt_callback");

    APP_ERROR_CHECK_BOOL(evt != NULL);

    switch (evt->type)
    {
        case TS_EVT_SYNCHRONIZED:
            NRF_LOG_INFO("TS_EVT_SYNCHRONIZED");
            // ts_imu_trigger_enable();
            m_gpio_trigger_enabled = true;
            break;
        case TS_EVT_DESYNCHRONIZED:
            NRF_LOG_INFO("TS_EVT_DESYNCHRONIZED");
            ts_imu_trigger_disable();
            m_gpio_trigger_enabled = false;
            break;
        case TS_EVT_TRIGGERED:
            NRF_LOG_INFO("TS_EVT_TRIGGERED");
            {
            uint32_t tick_target;

            NRF_LOG_INFO("State; %d", ts_state_get());

            if (m_imu_trigger_enabled && m_gpio_trigger_enabled)
            {
                NRF_LOG_INFO("TS_EVT_TRIGGERED && m_imu_trigger_enabled");

                tick_target = evt->params.triggered.tick_target + sync_interval_int_time;

                uint32_t time;
                time = TIME_SYNC_TIMESTAMP_TO_USEC(tick_target) / 1000;
                // NRF_LOG_INFO("target   %d", tick_target);

                uint32_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
                if(err_code != NRF_SUCCESS)
                {
                    NRF_LOG_INFO("err_code: %d", err_code);
                }
                APP_ERROR_CHECK(err_code);

                uint64_t time_now_ticks;
                uint32_t time_now_msec;
                time_now_ticks = TIME_SYNC_MSEC_TO_TICK(TIME_SYNC_TIMESTAMP_TO_USEC(ts_timestamp_get_ticks_u64()));
                // time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

                if(tick_target <= (time_now_ticks/1000))
                {
                    tick_target = evt->params.triggered.tick_target + sync_interval_int_time;
                    NRF_LOG_INFO("TimeSync tick_target <= ticks_now");
                }
                NRF_LOG_INFO("now   %d  tick_start   %d  tick_target  %d  last_sync   %d", time_now_ticks/1000, evt->params.triggered.tick_start, evt->params.triggered.tick_target, evt->params.triggered.last_sync);

                NRF_LOG_INFO("tick_target:  %d", tick_target);
                if( (tick_target % 100) == 0)
                {
                    NRF_LOG_INFO("Multiple of 100 ticks detected");
                    nrf_gpio_pin_toggle(TIMESYNC_PIN);
                    // NRF_LOG_INFO("Sync pin toggle");
                }

                // Process IMU BLE packet for sending
                // Only send data when imu.stop is not called
                imu_send_data();
            }
            else
            {
                // Ensure pin is low when triggering is stopped
                nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

                nrf_gpio_pin_clear(TIMESYNC_PIN);

                NRF_LOG_INFO("Triggering stopped");
            }
            }
            break;
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

static void sync_timer_init(void)
{
    uint32_t err_code;

    // Debug pin:
    // nRF52-DK (PCA10040) Toggle P0.24 from sync timer to allow pin measurement
    // nRF52840-DK (PCA10056) Toggle P1.14 from sync timer to allow pin measurement
#if defined(BOARD_PCA10040)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(0, 24), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#elif defined(BOARD_PCA10056)
    nrf_gpiote_task_configure(3, NRF_GPIO_PIN_MAP(1, 14), NRF_GPIOTE_POLARITY_TOGGLE, NRF_GPIOTE_INITIAL_VALUE_LOW);
    nrf_gpiote_task_enable(3);
#else
#warning Debug pin not set
#endif

    ts_init_t init_ts =
    {
        .high_freq_timer[0] = NRF_TIMER3,
        .high_freq_timer[1] = NRF_TIMER4,
        .egu                = NRF_EGU3,
        .egu_irq_type       = SWI3_EGU3_IRQn,
        .evt_handler        = ts_evt_callback,
    };

    err_code = ts_init(&init_ts);
    APP_ERROR_CHECK(err_code);

	ts_rf_config_t rf_config =
	{
		.rf_chn = 80,
		.rf_addr = { 0xDE, 0xAD, 0xBE, 0xEF, 0x19 }
	};

    err_code = ts_enable(&rf_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("Started listening for beacons.\r\n");
    NRF_LOG_INFO("Press Button 1 to start transmitting sync beacons\r\n");
    NRF_LOG_INFO("GPIO toggling will begin when transmission has started.\r\n");
}




/**@brief Function for handling events from the BSP module.
 *
 * @param[in]   event   Event generated by button press.
 */
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            {
                static bool m_send_sync_pkt = false;

                if (m_send_sync_pkt)
                {
                    m_send_sync_pkt = false;
                    m_gpio_trigger_enabled = false;

                    bsp_board_leds_off();

                    err_code = ts_tx_stop();
                    APP_ERROR_CHECK(err_code);

                    NRF_LOG_INFO("Stopping sync beacon transmission!\r\n");
                }
                else
                {
                    m_send_sync_pkt = true;

                    bsp_board_leds_on();

                    err_code = ts_tx_start(1); // 1Hz
                    APP_ERROR_CHECK(err_code);

                    ts_gpio_trigger_enable();

                    NRF_LOG_INFO("Starting sync beacon transmission!\r\n");
                }
            }
            break;

        case BSP_EVENT_KEY_2:
        {
            uint64_t time_ticks;
            uint32_t time_usec;

            time_ticks = ts_timestamp_get_ticks_u64();
            time_usec = TIME_SYNC_TIMESTAMP_TO_USEC(time_ticks);

            NRF_LOG_INFO("Timestamp: %d us (%d, %d)", time_usec, time_usec / 1000000, time_usec / 1000);
            break;
        }
						
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;

        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            if (err_code != NRF_ERROR_INVALID_STATE)
            {
                APP_ERROR_CHECK(err_code);
            }
            break;

        case BSP_EVENT_WHITELIST_OFF:
            if (m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                if (err_code != NRF_ERROR_INVALID_STATE)
                {
                    APP_ERROR_CHECK(err_code);
                }
            }
            break;

        default:
            break;
    }
}


/**@brief   Function for handling app_uart events.
 *
 * @details This function will receive a single character from the app_uart module and append it to
 *          a string. The string will be be sent over BLE when the last character received was a
 *          'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
 */
/**@snippet [Handling the data received over UART] */
void uart_event_handle(app_uart_evt_t * p_event)
{
    static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
    static uint8_t index = 0;
    uint32_t       err_code;

    switch (p_event->evt_type)
    {
        case APP_UART_DATA_READY:
            UNUSED_VARIABLE(app_uart_get(&data_array[index]));
            index++;

            if ((data_array[index - 1] == '\n') ||
                (data_array[index - 1] == '\r') ||
                (index >= m_ble_nus_max_data_len))
            {
                if (index > 1)
                {
                    NRF_LOG_DEBUG("Ready to send data over BLE NUS");
                    NRF_LOG_HEXDUMP_DEBUG(data_array, index);

                    do
                    {
                        uint16_t length = (uint16_t)index;
                        err_code = ble_nus_data_send(&m_nus, data_array, &length, m_conn_handle);
                        if ((err_code != NRF_ERROR_INVALID_STATE) &&
                            (err_code != NRF_ERROR_RESOURCES) &&
                            (err_code != NRF_ERROR_NOT_FOUND))
                        {
                            APP_ERROR_CHECK(err_code);
                        }
                    } while (err_code == NRF_ERROR_RESOURCES);
                }

                index = 0;
            }
            break;

        case APP_UART_COMMUNICATION_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_communication);
            break;

        case APP_UART_FIFO_ERROR:
            APP_ERROR_HANDLER(p_event->data.error_code);
            break;

        default:
            break;
    }
}
/**@snippet [Handling the data received over UART] */


/**@brief  Function for initializing the UART module.
 */
/**@snippet [UART Initialization] */
static void uart_init(void)
{
    uint32_t                     err_code;
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = USR_RX_PIN_NUMBER,			// 26
        .tx_pin_no    = USR_TX_PIN_NUMBER,			// 27
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
#if defined (UART_PRESENT)
        .baud_rate    = NRF_UART_BAUDRATE_115200
#else
        .baud_rate    = NRF_UARTE_BAUDRATE_115200
#endif
    };

    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}
/**@snippet [UART Initialization] */


/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    // CHANGES
    // m_more_adv_uuids[0].type = m_tms.uuid_type;

    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // CHANGES
    // init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
    // init.srdata.uuids_complete.p_uuids  = m_adv_uuids;

    // Costum added for TMS: need to use "uuids_more_available" and scan responce packet "srdata" becaus when advertising both uuids, 
    // the advertising packet length becomes more than 31 (which is the max length)
    init.advdata.uuids_more_available.uuid_cnt = 1;
    init.advdata.uuids_more_available.p_uuids = &m_adv_uuids[0];

    ble_advdata_t srdata;

    init.srdata.uuids_more_available.uuid_cnt = 2;
    init.srdata.uuids_more_available.p_uuids = &m_adv_uuids[1];


    // m_more_adv_uuids[0].type = m_tms.uuid_type;

    // init.srdata.uuids_more_available.uuid_cnt = sizeof(m_more_adv_uuids)/ sizeof(m_more_adv_uuids[0]);
    // init.srdata.uuids_more_available.p_uuids = m_more_adv_uuids;
    // End Costum

    init.config.ble_adv_fast_enabled  = true;
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
    init.evt_handler = on_adv_evt;

    err_code = ble_advertising_init(&m_advertising, &init);
    APP_ERROR_CHECK(err_code);

    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}


/**@brief Function for initializing buttons and leds.
 *
 * @param[out] p_erase_bonds  Will be true if the clear bonding button was pressed to wake the application up.
 */
static void buttons_leds_init(bool * p_erase_bonds)
{
    bsp_event_t startup_event;

    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
}




// Bool to keep track if data over BLE NUS is send correctly
bool NUS_send_OK = true;

// CUSTOM
uint32_t nus_printf_custom(char* p_char)
{
		static uint16_t index;
		uint32_t err_code;
		static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
		while(*p_char != '\0'){
						data_array[index] = *p_char;
						index++;
						p_char++;
						}

		err_code = ble_nus_data_send(&m_nus, data_array, &index, m_conn_handle);
		
		// Check for errors
		if ((err_code != NRF_ERROR_INVALID_STATE) &&
				(err_code != NRF_ERROR_RESOURCES) &&
				(err_code != NRF_ERROR_NOT_FOUND))
		{
				APP_ERROR_CHECK(err_code);
		}
				
		index = 0;
		return err_code;
}

// CUSTOM
uint32_t nus_printf_custom_1(char* p_char)
{
	static uint16_t index;
	uint32_t err_code;
	static uint8_t data_array[BLE_NUS_MAX_DATA_LEN];
	while(*p_char != '\0'){
          data_array[index] = *p_char;
          index++;
          p_char++;
          }			
			do
			{
					err_code = ble_nus_data_send(&m_nus, data_array, &index, m_conn_handle);
					if ((err_code != NRF_ERROR_INVALID_STATE) &&
							(err_code != NRF_ERROR_RESOURCES) &&
							(err_code != NRF_ERROR_NOT_FOUND))
					{
							APP_ERROR_CHECK(err_code);
					}
			} while (err_code == NRF_SUCCESS);
		
			NRF_LOG_INFO("C:	%d", countrrr);
	countrrr++;
		index = 0;
	return err_code;
}

uint32_t nus_send(uint8_t * data, uint16_t len)
{
		uint32_t err_code;
	
		err_code = ble_nus_data_send(&m_nus, data, &len, m_conn_handle);
		
		// Check for errors
		if ((err_code != NRF_ERROR_INVALID_STATE) &&
				(err_code != NRF_ERROR_RESOURCES) &&
				(err_code != NRF_ERROR_NOT_FOUND))
		{
				APP_ERROR_CHECK(err_code);
		}
		return err_code;
}





uint32_t usr_ble_init(void)
{
    bool erase_bonds;
    uint32_t err_code;

    uart_init();
    log_init();
    timers_init();
    buttons_leds_init(&erase_bonds);
    power_management_init();

    ble_stack_init();
    gap_params_init();
    gatt_init();
    
    // Init NUS services and standard services
    services_init();

    NRF_LOG_DEBUG("DEBUG enabled in preprocessor");
    NRF_LOG_FLUSH();

    // Init TMS service
    err_code = usr_tms_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("usr_tms_init done");

    err_code = usr_bas_init();
    APP_ERROR_CHECK(err_code);
    NRF_LOG_DEBUG("usr_bas_init done");

    advertising_init();
    NRF_LOG_DEBUG("advertising_init done");
    conn_params_init();
    NRF_LOG_DEBUG("conn_params_init done");

    // TimeSync
    sync_timer_init();
    NRF_LOG_DEBUG("sync_timer_button_init done");

    advertising_start();
    NRF_LOG_DEBUG("advertising_start done");

    return NRF_SUCCESS;
}


