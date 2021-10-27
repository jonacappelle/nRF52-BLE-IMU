#include "usr_wdt.h"

// Logging
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#define NRF_LOG_MODULE_NAME usr_wdt_c
#include "nrf_log.h"
NRF_LOG_MODULE_REGISTER();

bool wdt_wakeup = 0;

nrf_drv_wdt_channel_id m_channel_id;

/**
 * @brief WDT events handler.
 */
void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs

    wdt_wakeup = 1;

    // NRF_LOG_INFO("WDT evt_hanlder");
    // NRF_LOG_FLUSH();

}

bool is_wdt_wakeup()
{
    return wdt_wakeup;
}

void reset_wdt_wakeupt()
{
    NRF_LOG_INFO("Reset wdt_wakeup");
    wdt_wakeup = 0;
}

void wdt_init()
{
    ret_code_t err_code;

    //Configure WDT.
    nrf_drv_wdt_config_t config = NRF_DRV_WDT_DEAFULT_CONFIG;
    err_code = nrf_drv_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrf_drv_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrf_drv_wdt_enable();
}

void feed_wdt()
{
    nrf_drv_wdt_channel_feed(m_channel_id);
    // NRF_LOG_INFO("Feed WDT");
}




