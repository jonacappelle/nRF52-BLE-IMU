#include "usr_ble.h"

// Include utilities
#include "usr_util.h"

#include "usr_ble_settings.h"

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



#define NRF_LOG_MODULE_NAME usr_ble
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"


#include "usr_gpio.h"

// TimeSync
#include "time_sync.h"
#include "nrf_gpiote.h"
#include "nrf_ppi.h"
#include "nrf_timer.h"

// Timers
#include "usr_tmr.h"

// IMU params
#include "imu_params.h"
#include "imu.h"
extern imu_data_t imu_data;

// Application scheduler
#include "app_scheduler.h"

// Add TMS service for transmitting IMU data
#include "ble_tms.h"

// BLE Battery service
#include "ble_bas.h"

// BLE NUS Service
#include "usr_ble_nus.h"

// ADC
#include "usr_adc.h"



#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "nrf_dfu_ble_svci_bond_sharing.h"
#include "nrf_svci_async_function.h"
#include "nrf_svci_async_handler.h"

#include "nordic_common.h"
#include "nrf.h"
#include "app_error.h"
#include "ble.h"
#include "ble_hci.h"
#include "ble_srv_common.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_params.h"
#include "nrf_sdh.h"
#include "nrf_sdh_soc.h"
#include "nrf_sdh_ble.h"
#include "app_timer.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "bsp_btn_ble.h"
#include "ble_hci.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_conn_state.h"
#include "ble_dfu.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_qwr.h"
#include "fds.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_clock.h"
#include "nrf_power.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_bootloader_info.h"


extern bool calibration_timer_running;

ble_tms_config_t tms_cfg;

static bool calibration_started = false;

// Struct to keep track of TimeSync variables
typedef struct{
    uint32_t sync_interval_int_time; // How much time between measurements - (In increments of 2.5 ms)
    bool m_ts_synchronized;
    bool m_imu_trigger_enabled;
    bool m_ts_packet_received;
} time_sync_t;



time_sync_t ts = {
    .sync_interval_int_time = 4, // Default 100 Hz transmission rate
    .m_ts_synchronized = 0,
    .m_imu_trigger_enabled = 0,
    .m_ts_packet_received = 0,
};

ts_rf_config_t rf_config =
{
    .rf_chn = 80,
    .rf_addr = { 0xDE, 0xAD, 0xBE, 0xEF, 0x19 }
};


// Initialization
BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                                   /**< BLE NUS service instance. */
NRF_BLE_GATT_DEF(m_gatt);                                                           /**< GATT module instance. */
NRF_BLE_QWR_DEF(m_qwr);                                                             /**< Context for the Queued Write module.*/
BLE_ADVERTISING_DEF(m_advertising);                                                 /**< Advertising module instance. */

BLE_BAS_DEF(m_bas);                                                                 /**< Structure used to identify the battery service. */
ble_tms_t m_tms;                                                                    /**< Motion service */

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;                 /**< Handle of the current connection. */
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */
//static uint16_t   m_ble_nus_max_data_len = 247 - 3;            /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module. */


static ble_uuid_t m_adv_uuids[]          =                                          /**< Universally unique service identifier. */
{
    {BLE_UUID_NUS_SERVICE, NUS_SERVICE_UUID_TYPE},
    {BLE_UUID_TMS_SERVICE, TMS_SERVICE_UUID_TYPE}, // Added for TMS service
    {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE}
};


// Function prototypes
static bool ts_evt_synchronized();



void ble_send_quat(ble_tms_quat_t * data)
{
    ret_code_t err_code;

    err_code = ble_tms_quat_set(&m_tms, data);
    if(err_code != NRF_SUCCESS)
    {
        if(err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_INFO("Packet lost");
        }else{
            NRF_LOG_INFO("ble_tms_quat_set err_code: %d", err_code);
            APP_ERROR_CHECK(err_code);            
        }
    }
}

void ble_send_raw(ble_tms_raw_t * data)
{
    ret_code_t err_code;

    err_code = ble_tms_raw_set(&m_tms, data);	
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ble_tms_raw_set err_code: %d", err_code);
        APP_ERROR_CHECK(err_code);
    }    
}

void ble_send_adc(ble_tms_adc_t * data)
{
    ret_code_t err_code;

    err_code = ble_tms_adc_set(&m_tms, data);
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ble_tms_adc_set err_code: %d", err_code);
    } 
}

void ble_send_euler(ble_tms_euler_t * data)
{
    ret_code_t err_code;

    err_code = ble_tms_euler_set(&m_tms, data);
    if(err_code != NRF_SUCCESS)
    {
        NRF_LOG_INFO("ble_tms_euler_set err_code: %d", err_code);
    }
}


// Print received config
static void print_config(ble_tms_config_t* config)
{
    NRF_LOG_INFO("---RECEIVED CONFIG---")

    if(config->gyro_enabled) NRF_LOG_INFO("Gyro enabled");
    if(config->accel_enabled) NRF_LOG_INFO("Accel enabled");
    if(config->mag_enabled) NRF_LOG_INFO("Mag enabled");
    if(config->euler_enabled) NRF_LOG_INFO("Euler enabled");
    if(config->quat6_enabled) NRF_LOG_INFO("QUAT6 enabled");
    if(config->quat9_enabled) NRF_LOG_INFO("QUAT9 enabled");
    if(config->start_calibration) NRF_LOG_INFO("Calibration start enabled");
    NRF_LOG_INFO("Frequency: %d", config->motion_freq_hz);

    if(config->adc_enabled) NRF_LOG_INFO("ADC enabled");

    if(config->sync_enabled) NRF_LOG_INFO("SYNC enabled");
    NRF_LOG_INFO("Sync start time: %d", config->sync_start_time);

    if(config->wom_enabled) NRF_LOG_INFO("WoM enabled");

    NRF_LOG_FLUSH();
}



void sync_notif(void * p_event_data, uint16_t event_size)
{
    // Notify DCU that time is synced
    if(ts_evt_synchronized())
    {
        NRF_LOG_INFO("sync_info_send conn_handle: %d", m_conn_handle);
        sync_info_send(true);
    }
}


// Motion service event handler
static void ble_tms_evt_handler(ble_tms_t        * p_tms,
                                ble_tms_evt_type_t evt_type,
                                uint8_t          * p_data,
                                uint16_t           length)
{
    NRF_LOG_DEBUG("ble_tms_evt_handler called: %d", evt_type);
    ret_code_t err_code;
    
    switch (evt_type)
    {
        case BLE_TMS_EVT_NOTIF_TAP:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_TAP - %d\r\n", p_tms->is_tap_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_ADC:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_ADC - %d\r\n", p_tms->is_adc_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_QUAT:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_QUAT - %d\r\n", p_tms->is_quat_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_PEDOMETER:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_PEDOMETER - %d\r\n", p_tms->is_pedo_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_RAW:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_RAW - %d\r\n", p_tms->is_raw_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_EULER:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_EULER - %d\r\n", p_tms->is_euler_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_ROT_MAT:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_ROT_MAT - %d\r\n", p_tms->is_rot_mat_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_HEADING:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_HEADING - %d\r\n", p_tms->is_heading_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_GRAVITY:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_GRAVITY - %d\r\n", p_tms->is_gravity_notif_enabled);
            break;

        case BLE_TMS_EVT_NOTIF_INFO:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_NOTIF_INFO - %d\r\n", p_tms->is_info_notif_enabled);

            // Pass change IMU settings to event handler
            err_code = app_sched_event_put(0, 0, sync_notif);
            APP_ERROR_CHECK(err_code);

            break;

        case BLE_TMS_EVT_CONFIG_RECEIVED:
            NRF_LOG_INFO("ble_tms_evt_handler: BLE_TMS_EVT_CONFIG_RECEIVED - %d\r\n", length);
            APP_ERROR_CHECK_BOOL(length == sizeof(ble_tms_config_t));

            // Init struct for storing received data
            memcpy(&tms_cfg, p_data, length);

            // If wake on motion command is received
            // - Break the BLE connection
            // - Turn of wake-on-motion
            // - Setup interrupt pin to wake-up microcontroller
            if (tms_cfg.wom_enabled)
            {
                NRF_LOG_INFO("in if() statement");
                ble_disconnect();
            }

            // Keep track of when calibration has been started
            calibration_started = tms_cfg.start_calibration;

            // Print out received config over RTT
            print_config(&tms_cfg);

            // Pass change IMU settings to event handler
            err_code = app_sched_event_put(&tms_cfg, sizeof(tms_cfg), imu_config_evt_sceduled);
            APP_ERROR_CHECK(err_code);
            break;

        default:
            break;
    }
}




void batt_level_update(uint8_t battery_level)
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


static void ts_print_sync_time()
{
    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_ticks;

    time_now_ticks = ts_timestamp_get_ticks_u64();
    time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;
    time_ticks = TIME_SYNC_MSEC_TO_TICK(time_now_msec);
    NRF_LOG_INFO("Time: ticks %d - ms %d", time_ticks, time_now_msec);
}

static void ts_set_triggered_period(ble_tms_config_t* self)
{
    if(self->motion_freq_hz == 0)
    {
        NRF_LOG_INFO("Wrong frequency selected - set to default 0Hz");
        ts.sync_interval_int_time = (FREQ_TO_MS(100) / TIME_SYNC_TIMER_PERIOD_MS);
    }else{
        ts.sync_interval_int_time = (FREQ_TO_MS(self->motion_freq_hz) / TIME_SYNC_TIMER_PERIOD_MS);
    }
}

static void ts_start_trigger(ble_tms_config_t* p_evt)
{
    ret_code_t err_code;

    // Set timesync pin low
    led_off();

    // Print start time
    NRF_LOG_INFO("time.sync_start_time %d", p_evt->sync_start_time);

    if( ( p_evt->sync_enabled ) && ( p_evt->sync_start_time != 0) )
    {
        err_code = ts_set_trigger(p_evt->sync_start_time, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
        APP_ERROR_CHECK(err_code);
        ts.m_imu_trigger_enabled = 1 ;
    }else{
        ts.m_imu_trigger_enabled = 0;
    }
    NRF_LOG_INFO("ts.sync_interval_int_time: %d", ts.sync_interval_int_time);
}

// Event handler DIY
/**@brief GPIOTE sceduled handler, executed in main-context.
 */
void imu_config_evt_sceduled(void * p_event_data, uint16_t event_size)
{
		// Enable sensor parameters based on received configuration
		ret_code_t err_code;

        ble_tms_config_t config;
        memcpy(&config, p_event_data, event_size);

        // Clear all buffers before starting a new measurement
        imu_clear_buff();

        #if IMU_ENABLED == 1
		err_code =  imu_enable_sensors(&config);
		APP_ERROR_CHECK(err_code);
        #endif

        if( !config.wom_enabled && !config.start_calibration )
        {
            // Set correct trigger period for TS_EVT_TRIGGERED
            ts_set_triggered_period(&config);

            // Temp for debugging
            ts_print_sync_time();

            // Set trigger if peripheral is synced
            ts_start_trigger(&config);

            // TODO: ADC needs to be implemented
            // adc_enable(imu);
        }
}


void ble_disconnect(void)
{
    ret_code_t err_code;

    if (m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        // NRF_LOG_INFO("m_conn_handle in sd_ble_gap_disconnect %d", m_conn_handle);
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        // NRF_LOG_INFO("m_conn_handle in sd_ble_gap_disconnect %d", m_conn_handle);
        APP_ERROR_CHECK(err_code);
    }
}


/**@snippet [Handling the data received over BLE] */
static void usr_qwr_init()
{
    uint32_t           err_code;
    nrf_ble_qwr_init_t qwr_init = {0};

    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;

    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
}


static void usr_tms_init(void)
{
    ret_code_t err_code;
    ble_tms_init_t        tms_init;

    // Initialize Motion service
    memset(&tms_init, 0, sizeof(tms_init));

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

    err_code = ble_tms_init(&m_tms, &tms_init);
    APP_ERROR_CHECK(err_code);
}

static void usr_bas_init(void)
{
    ret_code_t err_code;
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


/**@brief Function for initializing services that will be used by the application.
 */
static void services_init(void)
{
    // Initialize Queued write module: for write requests (used by NUS + TMS)
    usr_qwr_init();

    // Initialize BLE NUS module
    usr_ble_nus_init();

    // Init TMS service
    usr_tms_init();

    // Initialize Battery service
    usr_bas_init();

    // Init DFU service
    // ble_dfu_init();
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
    ret_code_t err_code;

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
void sleep_mode_enter(void)
{
    ret_code_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);

    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);

    //Disable SoftDevice. It is required to be able to write to GPREGRET2 register (SoftDevice API blocks it).
    //GPREGRET2 register holds the information about skipping CRC check on next boot.
    err_code = nrf_sdh_disable_request();
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
    ret_code_t err_code;

    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            NRF_LOG_DEBUG("Fast Advertising");
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
        case BLE_ADV_EVT_IDLE:
            NRF_LOG_DEBUG("Advertising idle");

            // TODO need to enter sleep mode
            // sleep_mode_enter();
            // idle_state_handle();

            err_code = app_sched_event_put(0, 0, sleep);
            APP_ERROR_CHECK(err_code);
            break;
        default:
            break;
    }
}

static void imu_clear_struct(ble_tms_config_t* data)
{
    memset(data, 0, sizeof(data));
}

void sleep(void * p_event_data, uint16_t event_size)
{
    ret_code_t err_code;

    // Clear IMU params
    imu_clear_struct(&tms_cfg);

    // Clear all buffers before starting a new measurement
    imu_clear_buff();

    // De-initialize LED
    led_deinit();

    // Stop ts timer - otherwise when timesync packets are enabled and peripheral goes to sleep, there will be an event generated in 10 sec re-enabling the TimeSync receiver
    ts_timer_stop();

    // Disable time synchronization
    err_code = ts_disable();
    APP_ERROR_CHECK(err_code);

    err_code = ts_deinit();
    APP_ERROR_CHECK(err_code);

    // Attempt to busy wait until timeslot is closed
    do{
        // NRF_LOG_INFO("ts_timeslot_open");
        // NRF_LOG_FLUSH();
        idle_state_handle(); // Sleep while waiting
    }
    while( ts_timeslot_open() == 1 );
    NRF_LOG_INFO("ts timeslot closed");
    NRF_LOG_FLUSH();

    // Stop advertising - otherwise, a new connection will be made
    // When waking up from IMU WoM interrupt, we need to start advertising again
    advertising_stop();
    NRF_LOG_INFO("advertising_stop");
    NRF_LOG_FLUSH();

    // When in calibration mode - exit calibration mode when going to sleep
    stop_calibration_timer();

    // Shutdown IMU and enable WoM
    imu_sleep_wom();
    NRF_LOG_INFO("imu_sleep_wom");
    NRF_LOG_FLUSH();

    // Shutdown ADC
    usr_adc_deinit();
    NRF_LOG_INFO("usr_adc_deinit");
    NRF_LOG_FLUSH();

    NRF_LOG_INFO("Ready to go to sleep!");
    NRF_LOG_FLUSH();
}




/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    ret_code_t err_code;

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

            // Set transmit power to +4dBm
            err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_CONN, m_conn_handle, RADIO_TXPOWER_TXPOWER_Pos4dBm);
            APP_ERROR_CHECK(err_code);
            
            break;

        case BLE_GAP_EVT_DISCONNECTED:
            NRF_LOG_INFO("Disconnected");
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;

            err_code = app_sched_event_put(0, 0, sleep);
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
    ble_cfg.conn_cfg.params.gatts_conn_cfg.hvn_tx_queue_size = 20;              // Change hvn_tx_queue_size to 20: now we have the ability to queue up to 20 packets
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



// static void ts_gpio_trigger_enable(void)
// {
//     uint64_t time_now_ticks;
//     uint32_t time_now_msec;
//     uint32_t time_target;
//     uint32_t err_code;

//     if (m_gpio_trigger_enabled)
//     {
//         return;
//     }

//     // Round up to nearest second to next 2000 ms to start toggling.
//     // If the receiver has received a valid sync packet within this time, the GPIO toggling polarity will be the same.

//     time_now_ticks = ts_timestamp_get_ticks_u64();
//     time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

//     time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + (2000 * 2);
//     time_target = (time_target / 2000) * 2000;

//     err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
//     APP_ERROR_CHECK(err_code);

//     nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

//     m_gpio_trigger_enabled = true;
// }

// static void ts_gpio_trigger_disable(void)
// {
//     m_gpio_trigger_enabled = false;
// }


void ts_imu_trigger_enable(void)
{
    NRF_LOG_INFO("ts_imu_trigger_enable()");

    uint64_t time_now_ticks;
    uint32_t time_now_msec;
    uint32_t time_target;
    ret_code_t err_code;

    if (ts.m_imu_trigger_enabled)
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

    ts.m_imu_trigger_enabled = true;
}

static void ts_imu_trigger_disable(void)
{
    NRF_LOG_INFO("ts_imu_trigger_disable()");
    ts.m_imu_trigger_enabled = false;
}

void timesync_pin_toggle(uint32_t tick)
{
    // Toggle on multiples of 100 ticks
    if( (tick % 1000) == 0)
    {
        led_toggle();
    }
}


static void ts_evt_synchronized_enable()
{
    ts.m_ts_synchronized = 1;
}

static void ts_evt_synchronized_disable()
{
    ts.m_ts_synchronized = 0;
}

static bool ts_evt_synchronized()
{
    return ts.m_ts_synchronized;
}

void TimeSync_re_enable()
{
    ret_code_t err_code;

    err_code =  ts_re_enable(&rf_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("ts_re_enable");
}

void TimeSync_enable()
{
    ret_code_t err_code;

    err_code =  ts_enable(&rf_config);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("ts_enable");
}


static void ts_evt_callback(const ts_evt_t* evt)
{
    // NRF_LOG_INFO("ts_evt_callback");

    APP_ERROR_CHECK_BOOL(evt != NULL);

    ret_code_t err_code;

    switch (evt->type)
    {
        case TS_EVT_SYNCHRONIZED:
            NRF_LOG_INFO("TS_EVT_SYNCHRONIZED");
            NRF_LOG_FLUSH();
            // ts_imu_trigger_enable();
            ts_evt_synchronized_enable();

            uint64_t time_now_ticks;
            uint32_t time_now_msec;

            time_now_ticks = ts_timestamp_get_ticks_u64();
            time_now_msec = TIME_SYNC_TIMESTAMP_TO_USEC(time_now_ticks) / 1000;

            uint32_t time_target;
            time_target = TIME_SYNC_MSEC_TO_TICK(time_now_msec) + 100;
            time_target = (time_target / 100) * 100;

            err_code = ts_set_trigger(time_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
            APP_ERROR_CHECK(err_code);

            // Let DCU know to state of sync of different sensors
            sync_info_send(true);

            break;
        case TS_EVT_DESYNCHRONIZED:
            NRF_LOG_INFO("TS_EVT_DESYNCHRONIZED");
            led_off();
            ts_imu_trigger_disable();
            ts_evt_synchronized_disable();

            // Let DCU know to state of sync of different sensors
            sync_info_send(false);
            
            break;
        case TS_EVT_SYNC_PACKET_RECEIVED:
            NRF_LOG_INFO("TS_EVT_SYNC_PACKET_RECEIVED");

            // Temp disable TimeSync for x seconds to disable power
            do{
                err_code = ts_temp_disable();
                
                // Problem here: Receiver can't be disabled when it's already disabled
                // NRF_ERROR_FORBIDDEN when this happens, just neglect this error code and we will be fine
                if(err_code != NRF_ERROR_BUSY && err_code != NRF_ERROR_FORBIDDEN) APP_ERROR_CHECK(err_code);
            }while(err_code == NRF_ERROR_BUSY);

            ts_start_idle_timer(10);

            break;
        case TS_EVT_TRIGGERED:
            // NRF_LOG_INFO("TS_EVT_TRIGGERED");
            {
                uint32_t tick_target;

                if (ts.m_imu_trigger_enabled && ts_evt_synchronized())
                {
                    tick_target = evt->params.triggered.tick_target + ts.sync_interval_int_time;
                    // NRF_LOG_INFO("ts.sync_interval_int_time: %d", ts.sync_interval_int_time);

                    uint32_t time;
                    time = TIME_SYNC_TIMESTAMP_TO_USEC(tick_target) / 1000;

                    ret_code_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
                    if(err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_INFO("err_code: %d", err_code);
                    }
                    APP_ERROR_CHECK(err_code);

                    uint64_t time_now_ticks;
                    uint32_t time_now_msec;
                    time_now_ticks = TIME_SYNC_MSEC_TO_TICK(TIME_SYNC_TIMESTAMP_TO_USEC(ts_timestamp_get_ticks_u64()));

                    // Check for wrong timesync packet
                    if(tick_target <= (time_now_ticks/1000))
                    {
                        tick_target = evt->params.triggered.tick_target + ts.sync_interval_int_time;
                        NRF_LOG_INFO("TimeSync tick_target <= ticks_now");
                    }
                    // Print Time - Last Sync
                    // NRF_LOG_INFO("now   %d  tick_target  %d  last_sync   %d", time_now_ticks/1000, evt->params.triggered.tick_target, evt->params.triggered.last_sync);

                    // Toggle LED to measure TimeSync
                    timesync_pin_toggle(tick_target);

                    // Process IMU BLE packet for sending
                    imu_send_data(&tms_cfg);
                }
                else if (ts_evt_synchronized()) // When synchronized but not yet measuring
                {

                    // NRF_LOG_INFO("Synchronized LED toggle");

                    tick_target = evt->params.triggered.tick_target + 100;

                    ret_code_t err_code = ts_set_trigger(tick_target, nrf_gpiote_task_addr_get(NRF_GPIOTE_TASKS_OUT_3));
                    if(err_code != NRF_SUCCESS)
                    {
                        NRF_LOG_INFO("err_code: %d", err_code);
                    }
                    APP_ERROR_CHECK(err_code);

                    // Only toggle LED when calibration is not running
                    if(!calibration_timer_running)
                    {
                        led_toggle();
                    }


                }
                else
                {
                    // Ensure pin is low when triggering is stopped
                    nrf_gpiote_task_set(NRF_GPIOTE_TASKS_CLR_3);

                    // Set pin low when triggering is stopped
                    led_off();

                    NRF_LOG_INFO("Triggering stopped");
                }
            }
            break;
        default:
            APP_ERROR_CHECK_BOOL(false);
            break;
    }
}

void sync_timer_init(void)
{
    ret_code_t err_code;

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
    ret_code_t err_code;
    switch (event)
    {
        case BSP_EVENT_KEY_0:
            NRF_LOG_INFO("Key pressed");
            imu_set_config();

            break;

        case BSP_EVENT_KEY_2:
        {
            // Print syncrhonized timestamp
            ts_print_sync_time();
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




// Variable to keep track if the NUS buffer is full
static bool nus_buffer_full = false;


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
    ret_code_t err_code;
	
    switch (p_evt->type)
    {
        case BLE_NUS_EVT_RX_DATA:				
                // Pass change IMU settings to event handler - no RX functionality needed in slave
                // err_code = app_sched_event_put(0, 0, imu_config_evt_sceduled);
                // APP_ERROR_CHECK(err_code);
            break;
                
            // Added //
            // When BLE NUS
        case BLE_NUS_EVT_TX_RDY:
                nus_buffer_full = false;
//				NRF_LOG_INFO("BLE_NUS_EVT_TX_RDY");
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

void usr_ble_nus_init()
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;

    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));

    nus_init.data_handler = nus_data_handler;

    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);    
}



// CUSTOM
uint32_t nus_printf_custom(char* p_char)
{
		static uint16_t index;
		ret_code_t err_code;
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
	ret_code_t err_code;
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
		index = 0;
	return err_code;
}

// Send data over NUS service
uint32_t nus_send(uint8_t * data, uint16_t len)
{
		ret_code_t err_code;
	
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
void advertising_init(void)
{
    uint32_t               err_code;
    ble_advertising_init_t init;

    memset(&init, 0, sizeof(init));

    // CHANGES
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;

    // Costum added for TMS: need to use "uuids_more_available" and scan responce packet "srdata" becaus when advertising both uuids, 
    // the advertising packet length becomes more than 31 (which is the max length)
    init.advdata.uuids_more_available.uuid_cnt = 1;
    init.advdata.uuids_more_available.p_uuids = &m_adv_uuids[0];

    ble_advdata_t srdata;

    init.srdata.uuids_more_available.uuid_cnt = 2;
    init.srdata.uuids_more_available.p_uuids = &m_adv_uuids[1];
    // End Costum

    advertising_config_get(&init.config);

    // init.config.ble_adv_fast_enabled  = true;
    // init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
    // init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;
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

    ret_code_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);

    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}


/**@brief Function for initializing the nrf log module.
 */
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();

    // Display start message
    NRF_LOG_INFO("Debug logging for UART over RTT started.");

    NRF_LOG_DEBUG("DEBUG enabled in preprocessor");
    NRF_LOG_FLUSH();
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for starting advertising.
 */
void advertising_start(void)
{
    ret_code_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("advertising_start");
}


void advertising_stop(void)
{   
    // @retval ::NRF_SUCCESS The BLE stack has stopped advertising.
    // @retval ::BLE_ERROR_INVALID_ADV_HANDLE Invalid advertising handle.
    // @retval ::NRF_ERROR_INVALID_STATE The advertising handle is not advertising.
    ret_code_t err_code;

	err_code = sd_ble_gap_adv_stop(m_advertising.adv_handle);
    
    if( err_code == NRF_ERROR_INVALID_STATE )
    {
        NRF_LOG_INFO("NRF_ERROR_INVALID_STATE");
    }
    if( err_code != NRF_ERROR_INVALID_STATE )
    {
        NRF_LOG_INFO("Advertising stopped");
        APP_ERROR_CHECK(err_code);
    }
}

void set_transmit_power_4dbm()
{
    ret_code_t err_code;
    err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, RADIO_TXPOWER_TXPOWER_Pos4dBm);
    APP_ERROR_CHECK(err_code);
}

void usr_ble_init(void)
{
    // UART Init - (not really necessary) -  used by BLE NUS
    // uart_init();

    // Initialize application timers
    timers_init();

    // Setup buttons + Leds on NRF_DK board
    // bool erase_bonds;
    // buttons_leds_init(&erase_bonds);

    // Init power management
    power_management_init();

    // Initialize BLE stack
    // ble_stack_init();

    // for DFU
    // peer_manager_init();

    // Init Gap connection parameters
    // gap_params_init();

    // Init GATT
    // gatt_init();
    
    // Init Queued write - NUS - Motion - Battery services
    // services_init();

    // Init Advertising
    // advertising_init();

    // Longer range for advertising - need to set this also in connection
    // set_transmit_power_4dbm();

    // Init connection parameters
    // conn_params_init();

    // Initialize time synchronization functionality
    // sync_timer_init();
    // ts_timer_init();

    // Start advertising
    // advertising_start();
}



void advertising_config_get(ble_adv_modes_config_t * p_config)
{
    memset(p_config, 0, sizeof(ble_adv_modes_config_t));

    p_config->ble_adv_fast_enabled  = true;
    p_config->ble_adv_fast_interval = APP_ADV_INTERVAL;
    p_config->ble_adv_fast_timeout  = APP_ADV_DURATION;
}

void dfu_async_init()
{
    ret_code_t err_code;
 
    // Initialize the async SVCI interface to bootloader before any interrupts are enabled.
    err_code = ble_dfu_buttonless_async_svci_init();
    APP_ERROR_CHECK(err_code);
}




static void buttonless_dfu_sdh_state_observer(nrf_sdh_state_evt_t state, void * p_context)
{
    if (state == NRF_SDH_EVT_STATE_DISABLED)
    {
        // Softdevice was disabled before going into reset. Inform bootloader to skip CRC on next boot.
        nrf_power_gpregret2_set(BOOTLOADER_DFU_SKIP_CRC);

        //Go to system off.
        nrf_pwr_mgmt_shutdown(NRF_PWR_MGMT_SHUTDOWN_GOTO_SYSOFF);
    }
}

/* nrf_sdh state observer. */
NRF_SDH_STATE_OBSERVER(m_buttonless_dfu_state_obs, 0) =
{
    .handler = buttonless_dfu_sdh_state_observer,
};

static void disconnect(uint16_t conn_handle, void * p_context)
{
    UNUSED_PARAMETER(p_context);

    ret_code_t err_code = sd_ble_gap_disconnect(conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    if (err_code != NRF_SUCCESS)
    {
        NRF_LOG_WARNING("Failed to disconnect connection. Connection handle: %d Error: %d", conn_handle, err_code);
    }
    else
    {
        NRF_LOG_DEBUG("Disconnected connection handle %d", conn_handle);
    }
}

// YOUR_JOB: Update this code if you want to do anything given a DFU event (optional).
/**@brief Function for handling dfu events from the Buttonless Secure DFU service
 *
 * @param[in]   event   Event from the Buttonless Secure DFU service.
 */
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            NRF_LOG_INFO("Device is preparing to enter bootloader mode.");

            // Prevent device from advertising on disconnect.
            ble_adv_modes_config_t config;
            advertising_config_get(&config);
            config.ble_adv_on_disconnect_disabled = true;
            ble_advertising_modes_config_set(&m_advertising, &config);

            // Disconnect all other bonded devices that currently are connected.
            // This is required to receive a service changed indication
            // on bootup after a successful (or aborted) Device Firmware Update.
            uint32_t conn_count = ble_conn_state_for_each_connected(disconnect, NULL);
            NRF_LOG_INFO("Disconnected %d links.", conn_count);
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            NRF_LOG_INFO("Device will enter bootloader mode.");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            NRF_LOG_ERROR("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            NRF_LOG_ERROR("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            APP_ERROR_CHECK(false);
            break;

        default:
            NRF_LOG_ERROR("Unknown event from ble_dfu_buttonless.");
            break;
    }
}



/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const * p_evt)
{
    pm_handler_on_pm_evt(p_evt);
    pm_handler_flash_clean(p_evt);
}

#define SEC_PARAM_BOND                  1                                           /**< Perform bonding. */
#define SEC_PARAM_MITM                  0                                           /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC                  0                                           /**< LE Secure Connections not enabled. */
#define SEC_PARAM_KEYPRESS              0                                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES       BLE_GAP_IO_CAPS_NONE                        /**< No I/O capabilities. */
#define SEC_PARAM_OOB                   0                                           /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE          7                                           /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE          16                                          /**< Maximum encryption key size. */

/**@brief Function for the Peer Manager initialization.
 */
void peer_manager_init()
{
    ble_gap_sec_params_t sec_param;
    ret_code_t           err_code;

    err_code = pm_init();
    APP_ERROR_CHECK(err_code);

    memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

    // Security parameters to be used for all security procedures.
    sec_param.bond           = SEC_PARAM_BOND;
    sec_param.mitm           = SEC_PARAM_MITM;
    sec_param.lesc           = SEC_PARAM_LESC;
    sec_param.keypress       = SEC_PARAM_KEYPRESS;
    sec_param.io_caps        = SEC_PARAM_IO_CAPABILITIES;
    sec_param.oob            = SEC_PARAM_OOB;
    sec_param.min_key_size   = SEC_PARAM_MIN_KEY_SIZE;
    sec_param.max_key_size   = SEC_PARAM_MAX_KEY_SIZE;
    sec_param.kdist_own.enc  = 1;
    sec_param.kdist_own.id   = 1;
    sec_param.kdist_peer.enc = 1;
    sec_param.kdist_peer.id  = 1;

    err_code = pm_sec_params_set(&sec_param);
    APP_ERROR_CHECK(err_code);

    err_code = pm_register(pm_evt_handler);
    APP_ERROR_CHECK(err_code);
}


/** @brief Clear bonding information from persistent storage.
 */
static void delete_bonds(void)
{
    ret_code_t err_code;

    NRF_LOG_INFO("Erase bonds!");

    err_code = pm_peers_delete();
    APP_ERROR_CHECK(err_code);
}


void ble_dfu_init()
{
    ret_code_t err_code;

    ble_dfu_buttonless_init_t dfus_init = {0};

    dfus_init.evt_handler = ble_dfu_evt_handler;

    err_code = ble_dfu_buttonless_init(&dfus_init);
    APP_ERROR_CHECK(err_code);
}


void send_calibration(bool start, bool gyro_done, bool accel_done, bool mag_done)
{
    ret_code_t err_code;

    ble_tms_info_t data;

    // Initialize struct to all zeros
    memset(&data, 0, sizeof(data));

    data.calibration_start = start;

    // Load calibration info data
    data.gyro_calibration_done = gyro_done;
    data.accel_calibration_drone = accel_done;
    data.mag_calibration_done = mag_done;
    
    if(data.gyro_calibration_done && data.accel_calibration_drone && data.mag_calibration_done)
    {
        data.calibration_done = 1;
    }

    err_code = ble_tms_info_set(&m_tms, &data);
    if(err_code != NRF_SUCCESS)
    {
        if(err_code == NRF_ERROR_RESOURCES)
        {
            NRF_LOG_INFO("Packet lost");
        }else{
            NRF_LOG_INFO("ble_tms_info_set err_code: %d", err_code);
            APP_ERROR_CHECK(err_code);            
        }
    }
    APP_ERROR_CHECK(err_code);

    NRF_LOG_INFO("---");
    NRF_LOG_INFO("Send Calibration Packet");
    NRF_LOG_INFO("---");
    NRF_LOG_INFO("Gyro: %d", data.gyro_calibration_done);
    NRF_LOG_INFO("Accel: %d", data.accel_calibration_drone);
    NRF_LOG_INFO("Mag: %d", data.mag_calibration_done);
    NRF_LOG_INFO("---");
}

void sync_info_send(bool state)
{
    ret_code_t err_code;

    ble_tms_info_t data;

    // Initialize struct to all zeros
    memset(&data, 0, sizeof(data));

    if(state)
    {
        data.sync_complete = true;
    }else{
        data.sync_lost = true;
    }

    NRF_LOG_INFO("sync_info_send conn_handle: %d", m_conn_handle);
    NRF_LOG_INFO("sync_info_send conn_handle enabled: %d", m_tms.is_info_notif_enabled);

    // Send packet only when in a connection
    if(m_conn_handle != BLE_CONN_HANDLE_INVALID)
    {
        err_code = ble_tms_info_set(&m_tms, &data);
        if(err_code != NRF_SUCCESS)
        {
            if(err_code == NRF_ERROR_RESOURCES)
            {
                NRF_LOG_INFO("Packet lost");
            }else{
                NRF_LOG_INFO("ble_tms_info_set err_code: %d", err_code);
                APP_ERROR_CHECK(err_code);            
            }
        }
        APP_ERROR_CHECK(err_code);

        NRF_LOG_INFO("---");
        NRF_LOG_INFO("Send Sync Packet Info");
        NRF_LOG_INFO("---");
        NRF_LOG_INFO("Sync: %d", data.sync_complete);
        NRF_LOG_INFO("---");
    }
}
