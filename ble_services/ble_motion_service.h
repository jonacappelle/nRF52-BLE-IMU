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
 *         File: ble_motion_service.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Bluetooth Low Energy - Services and Characteristics Description
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _BLE_MOTION_SERVICE_H__
#define _BLE_MOTION_SERVICE_H__

#include "sdk_common.h"
#include "ble.h"
#include "ble_srv_common.h"
#include "app_util_platform.h"
#include <stdint.h>
#include <stdbool.h>

#define BLE_TMS_BLE_OBSERVER_PRIO       2       //--> BLE TMS event gets send with high priority


// How many packets (QUAT - RAW) are grouped in a message
#define BLE_PACKET_BUFFER_COUNT     5


/**@brief Macro for defining a ble_tms instance.
 *
 * @param   _name  Name of the instance.
 * @hideinitializer
 */
#define BLE_TMS_DEF(_name)                          \
    static ble_tms_t _name;                         \
    NRF_SDH_BLE_OBSERVER(_name ## _obs,             \
                         BLE_TMS_BLE_OBSERVER_PRIO, \
                         ble_tms_on_ble_evt,        \
                         &_name)


#define BLE_UUID_TMS_SERVICE 0x0400                      /**< The UUID of the Motion Service. */
// #define BLE_TMS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - 3) /**< Maximum length of data (in bytes) that can be transmitted to the peer by the Motion service module. */

#define OPCODE_LENGTH        1
#define HANDLE_LENGTH        2

#if defined(NRF_SDH_BLE_GATT_MAX_MTU_SIZE) && (NRF_SDH_BLE_GATT_MAX_MTU_SIZE != 0)
    #define BLE_TMS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
#else
    #define BLE_TMS_MAX_DATA_LEN (BLE_GATT_ATT_MTU_DEFAULT - OPCODE_LENGTH - HANDLE_LENGTH)
    #warning NRF_SDH_BLE_GATT_MAX_MTU_SIZE is not defined.
#endif

#ifdef __GNUC__
    #ifdef PACKED
        #undef PACKED
    #endif

    #define PACKED(TYPE) TYPE __attribute__ ((packed))
#endif

typedef PACKED( struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}) ble_tms_raw_accel_t;

typedef PACKED( struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}) ble_tms_raw_gyro_t;

typedef PACKED( struct
{
    int16_t x;
    int16_t y;
    int16_t z;
}) ble_tms_raw_compass_t;

typedef PACKED( struct
{
    ble_tms_raw_accel_t   accel;
    ble_tms_raw_gyro_t    gyro;
    ble_tms_raw_compass_t compass;
    uint32_t timestamp_ms;
}) ble_tms_single_raw_t;

typedef PACKED( struct
{
    ble_tms_single_raw_t raw[BLE_PACKET_BUFFER_COUNT];
}) ble_tms_raw_t;


typedef PACKED( struct
{
    uint32_t raw[40];
    uint32_t timestamp_ms;
})ble_tms_adc_t;

typedef PACKED( struct
{
    
    int32_t w;
    int32_t x;
    int32_t y;
    int32_t z;
    uint32_t timestamp_ms;
}) ble_tms_single_quat_t;

typedef PACKED( struct
{ 
    ble_tms_single_quat_t quat[BLE_PACKET_BUFFER_COUNT];
}) ble_tms_quat_t;


typedef PACKED( struct
{ 
    bool calibration_start;
    bool gyro_calibration_done;
    bool accel_calibration_drone;
    bool mag_calibration_done;
    bool calibration_done;
    bool sync_complete;
    bool sync_lost;
}) ble_tms_info_t;


typedef PACKED( struct
{
    int32_t roll;
    int32_t pitch;
    int32_t yaw;
    uint32_t timestamp_ms;
}) ble_tms_euler_t;

typedef struct
{
    bool                      gyro_enabled;
    bool                      accel_enabled;
    bool                      mag_enabled;
    bool                      euler_enabled;
    bool                      quat6_enabled;
    bool                      quat9_enabled;
    uint16_t                  motion_freq_hz;
    bool                      wom_enabled;
    bool                      sync_enabled;
    uint64_t                  sync_start_time;
    bool                      stop;
    bool                      adc_enabled;
    bool                      start_calibration;
} ble_tms_config_t;


typedef enum
{
    BLE_TMS_EVT_CONFIG_RECEIVED,
    BLE_TMS_EVT_NOTIF_ADC,
    BLE_TMS_EVT_NOTIF_QUAT,
    BLE_TMS_EVT_NOTIF_RAW,
    BLE_TMS_EVT_NOTIF_EULER,
    BLE_TMS_EVT_NOTIF_INFO,
}ble_tms_evt_type_t;

/* Forward declaration of the ble_tms_t type. */
typedef struct ble_tms_s ble_tms_t;

/**@brief Motion Service event handler type. */
typedef void (*ble_tms_evt_handler_t) (ble_tms_t        * p_tms,
                                       ble_tms_evt_type_t evt_type,
                                       uint8_t          * p_data,
                                       uint16_t           length);

/**@brief Motion Service initialization structure.
 *
 * @details This structure contains the initialization information for the service. The application
 * must fill this structure and pass it to the service using the @ref ble_tms_init function.
 */
typedef struct
{
    ble_tms_config_t      * p_init_config;
    ble_tms_evt_handler_t   evt_handler; /**< Event handler to be called for handling received data. */
} ble_tms_init_t;

/**@brief Motion Service structure.
 *
 * @details This structure contains status information related to the service.
 */
struct ble_tms_s
{
    uint8_t                  uuid_type;                    /**< UUID type for Motion Service Base UUID. */
    uint16_t                 service_handle;               /**< Handle of Motion Service (as provided by the S110 SoftDevice). */
    ble_gatts_char_handles_t config_handles;               /**< Handles related to the config characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t adc_handles;          /**< Handles related to the adc characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t quat_handles;                 /**< Handles related to the quaternion characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t raw_handles;                  /**< Handles related to the raw data characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t euler_handles;                /**< Handles related to the euler angles characteristic (as provided by the S132 SoftDevice). */
    ble_gatts_char_handles_t info_handles;
    uint16_t                 conn_handle;                  /**< Handle of the current connection (as provided by the S110 SoftDevice). BLE_CONN_HANDLE_INVALID if not in a connection. */
    bool                     is_adc_notif_enabled; /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_quat_notif_enabled;        /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_raw_notif_enabled;         /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_euler_notif_enabled;       /**< Variable to indicate if the peer has enabled notification of the characteristic.*/
    bool                     is_info_notif_enabled;
    ble_tms_evt_handler_t    evt_handler;                  /**< Event handler to be called for handling received data. */
};

/**@brief Function for initializing the Motion Service.
 *
 * @param[out] p_wss      Motion Service structure. This structure must be supplied
 *                        by the application. It is initialized by this function and will
 *                        later be used to identify this particular service instance.
 * @param[in] p_tms_init  Information needed to initialize the service.
 *
 * @retval NRF_SUCCESS If the service was successfully initialized. Otherwise, an error code is returned.
 * @retval NRF_ERROR_NULL If either of the pointers p_wss or p_tms_init is NULL.
 */
uint32_t ble_tms_init(ble_tms_t * p_wss, const ble_tms_init_t * p_tms_init);

/**@brief Function for handling the Motion Service's BLE events.
 *
 * @details The Motion Service expects the application to call this function each time an
 * event is received from the S110 SoftDevice. This function processes the event if it
 * is relevant and calls the Motion Service event handler of the
 * application if necessary.
 *
 * @param[in] p_wss       Motion Service structure.
 * @param[in] p_ble_evt   Event received from the S110 SoftDevice.
 */
void ble_tms_on_ble_evt(ble_tms_t * p_wss, ble_evt_t * p_ble_evt);

/**@brief Function for sending adc data.
 *
 * @details This function sends the input adc as an adc characteristic notification to the peer.
 *
 * @param[in] p_tms       Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the adc data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tms_adc_set(ble_tms_t * p_tms, ble_tms_adc_t * p_data);

/**@brief Function for sending quaternion data.
 *
 * @details This function sends the input quaternion as an quaternion characteristic notification to the peer.
 *
 * @param[in] p_tms       Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the quaternion data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tms_quat_set(ble_tms_t * p_tms, ble_tms_quat_t * p_data);

/**@brief Function for sending info data.
 *
 * @details This function sends the input quaternion as an quaternion characteristic notification to the peer.
 *
 * @param[in] p_tms       Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the quaternion data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tms_info_set(ble_tms_t * p_tms, ble_tms_info_t * p_data);

/**@brief Function for sending raw data.
 *
 * @details This function sends the input as an raw characteristic notification to the peer.
 *
 * @param[in] p_tms       Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tms_raw_set(ble_tms_t * p_tms, ble_tms_raw_t * p_data);

/**@brief Function for sending euler angle data.
 *
 * @details This function sends the input pitch, roll and yaw as an euler characteristic notification to the peer.
 *
 * @param[in] p_tms       Pointer to the Motion Service structure.
 * @param[in] p_data      Pointer to the data.
 *
 * @retval NRF_SUCCESS If the string was sent successfully. Otherwise, an error code is returned.
 */
uint32_t ble_tms_euler_set(ble_tms_t * p_tms, ble_tms_euler_t * p_data);

#endif // BLE_TMS_H__

/** @} */
