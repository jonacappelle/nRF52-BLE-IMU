#include "usr_flash.h"
#include "fds.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"

// Logging functionality
#define NRF_LOG_MODULE_NAME usr_flash
#define NRF_LOG_LEVEL 4
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"

/* Array to map FDS events to strings. */
static char const * fds_evt_str[] =
{
    "FDS_EVT_INIT",
    "FDS_EVT_WRITE",
    "FDS_EVT_UPDATE",
    "FDS_EVT_DEL_RECORD",
    "FDS_EVT_DEL_FILE",
    "FDS_EVT_GC",
};


/* File ID and Key used for the configuration record. */

#define CONFIG_FILE     (0x8010)
#define CONFIG_REC_KEY  (0x7010)

/* Flag to check fds initialization. */
static bool volatile m_fds_initialized;

/* Keep track of the progress of a delete_all operation. */
static struct
{
    bool delete_next;   //!< Delete next record.
    bool pending;       //!< Waiting for an fds FDS_EVT_DEL_RECORD event, to delete the next record.
} m_delete_all;

/* A dummy structure to save in flash. */
typedef struct
{
    uint32_t boot_count;
    char     device_name[16];
    bool     config1_on;
    bool     config2_on;
} configuration_t;

/* Dummy configuration data. */
static configuration_t m_dummy_cfg =
{
    .config1_on  = false,
    .config2_on  = true,
    .boot_count  = 0x0,
    .device_name = "dummy",
};

/* A record containing dummy configuration data. */
static fds_record_t const m_dummy_record =
{
    .file_id           = CONFIG_FILE,
    .key               = CONFIG_REC_KEY,
    .data.p_data       = &m_dummy_cfg,
    /* The length of a record is always expressed in 4-byte units (words). */
    .data.length_words = (sizeof(m_dummy_cfg) + 3) / sizeof(uint32_t),
};

const char *fds_err_str(ret_code_t ret)
{
    /* Array to map FDS return values to strings. */
    static char const * err_str[] =
    {
        "FDS_ERR_OPERATION_TIMEOUT",
        "FDS_ERR_NOT_INITIALIZED",
        "FDS_ERR_UNALIGNED_ADDR",
        "FDS_ERR_INVALID_ARG",
        "FDS_ERR_NULL_ARG",
        "FDS_ERR_NO_OPEN_RECORDS",
        "FDS_ERR_NO_SPACE_IN_FLASH",
        "FDS_ERR_NO_SPACE_IN_QUEUES",
        "FDS_ERR_RECORD_TOO_LARGE",
        "FDS_ERR_NOT_FOUND",
        "FDS_ERR_NO_PAGES",
        "FDS_ERR_USER_LIMIT_REACHED",
        "FDS_ERR_CRC_CHECK_FAILED",
        "FDS_ERR_BUSY",
        "FDS_ERR_INTERNAL",
    };

    return err_str[ret - NRF_ERROR_FDS_ERR_BASE];
}

/**@brief   Sleep until an event is received. */
static void power_manage(void)
{
#ifdef SOFTDEVICE_PRESENT
    (void) sd_app_evt_wait();
#else
    __WFE();
#endif
}

/**@brief   Wait for fds to initialize. */
static void wait_for_fds_ready(void)
{
    while (!m_fds_initialized)
    {
        power_manage();
    }
}

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    if (p_evt->result == NRF_SUCCESS)
    {
        NRF_LOG_INFO("Event: %s received (NRF_SUCCESS)",
                      fds_evt_str[p_evt->id]);
    }
    else
    {
        NRF_LOG_INFO("Event: %s received (%s)",
                      fds_evt_str[p_evt->id],
                      fds_err_str(p_evt->result));
    }

    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
                m_fds_initialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->write.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->write.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->write.record_key);
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
                NRF_LOG_INFO("Record ID:\t0x%04x",  p_evt->del.record_id);
                NRF_LOG_INFO("File ID:\t0x%04x",    p_evt->del.file_id);
                NRF_LOG_INFO("Record key:\t0x%04x", p_evt->del.record_key);
            }
            m_delete_all.pending = false;
        } break;

        default:
            break;
    }
}


void usr_flash_write(uint8_t const * data, uint32_t len)
{
    ret_code_t rc;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        // /* A config file is in flash. Let's update it. */
        // fds_flash_record_t config = {0};

        // /* Open the record and read its contents. */
        // rc = fds_record_open(&desc, &config);
        // APP_ERROR_CHECK(rc);

        // /* Copy the configuration from flash into m_dummy_cfg. */
        // memcpy(data, config.p_data, len);

        // NRF_LOG_INFO("Config file found, updating boot count to %d.", m_dummy_cfg.boot_count);

        // /* Update boot count. */
        // m_dummy_cfg.boot_count++;

        // /* Close the record when done reading. */
        // rc = fds_record_close(&desc);
        // APP_ERROR_CHECK(rc);

        static uint8_t temp[84];

        memcpy(temp, data, len);

        static const len1 = 84;


        /* A record containing dummy configuration data. */
        static fds_record_t const record =
        {
            .file_id           = CONFIG_FILE,
            .key               = CONFIG_REC_KEY,
            .data.p_data       = temp,
            /* The length of a record is always expressed in 4-byte units (words). */
            .data.length_words = len1 / sizeof(uint32_t),
        };


        /* Write the updated record to flash. */
        rc = fds_record_update(&desc, &record);
        if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        {
            NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        }
        else
        {
            APP_ERROR_CHECK(rc);
        }
    }
}

void usr_flash_read(uint8_t * data, uint32_t len)
{
    ret_code_t rc;

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
        /* A config file is in flash. Let's update it. */
        fds_flash_record_t config = {0};

        /* Open the record and read its contents. */
        rc = fds_record_open(&desc, &config);
        APP_ERROR_CHECK(rc);

        /* Copy the configuration from flash into m_dummy_cfg. */
        memcpy(data, config.p_data, len);

        NRF_LOG_INFO("Config file read");

        /* Close the record when done reading. */
        rc = fds_record_close(&desc);
        APP_ERROR_CHECK(rc);
    }
}

bool usr_flash_check_valid_record()
{
    ret_code_t rc;

    fds_stat_t stat = {0};

    rc = fds_stat(&stat);
    APP_ERROR_CHECK(rc);

    NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    if(stat.valid_records == 0)
    {
        return 0;
    }else
    {
        return 1;
    }
}


void usr_flash_init()
{
    ret_code_t rc;

    /* Register first to receive an event when initialization is complete. */
    (void) fds_register(fds_evt_handler);

    rc = fds_init();
    APP_ERROR_CHECK(rc);

    /* Wait for fds to initialize. */
    wait_for_fds_ready();

    // Collect garbage records
    fds_gc();


    // fds_stat_t stat = {0};

    // rc = fds_stat(&stat);
    // APP_ERROR_CHECK(rc);

    // NRF_LOG_INFO("Found %d valid records.", stat.valid_records);
    // NRF_LOG_INFO("Found %d dirty records (ready to be garbage collected).", stat.dirty_records);

    // fds_record_desc_t desc = {0};
    // fds_find_token_t  tok  = {0};

    // rc = fds_record_find(CONFIG_FILE, CONFIG_REC_KEY, &desc, &tok);

    // if (rc == NRF_SUCCESS)
    //     {
    //         /* A config file is in flash. Let's update it. */
    //         fds_flash_record_t config = {0};

    //         /* Open the record and read its contents. */
    //         rc = fds_record_open(&desc, &config);
    //         APP_ERROR_CHECK(rc);

    //         /* Copy the configuration from flash into m_dummy_cfg. */
    //         memcpy(&m_dummy_cfg, config.p_data, sizeof(configuration_t));

    //         NRF_LOG_INFO("Config file found, updating boot count to %d.", m_dummy_cfg.boot_count);

    //         /* Update boot count. */
    //         m_dummy_cfg.boot_count++;

    //         /* Close the record when done reading. */
    //         rc = fds_record_close(&desc);
    //         APP_ERROR_CHECK(rc);

    //         /* Write the updated record to flash. */
    //         rc = fds_record_update(&desc, &m_dummy_record);
    //         if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
    //         {
    //             NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
    //         }
    //         else
    //         {
    //             APP_ERROR_CHECK(rc);
    //         }
    //     }
        // else
        // {
        //     /* System config not found; write a new one. */
        //     NRF_LOG_INFO("Writing config file...");

        //     rc = fds_record_write(&desc, &m_dummy_record);
        //     if ((rc != NRF_SUCCESS) && (rc == FDS_ERR_NO_SPACE_IN_FLASH))
        //     {
        //         NRF_LOG_INFO("No space in flash, delete some records to update the config file.");
        //     }
        //     else
        //     {
        //         APP_ERROR_CHECK(rc);
        //     }
        // }
}


