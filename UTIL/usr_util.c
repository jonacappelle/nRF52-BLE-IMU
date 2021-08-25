#include "usr_util.h"

#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_pwr_mgmt.h"

// Application scheduler
#include "app_scheduler.h"

#include "nrf_drv_clock.h"
#include "nrf_pwr_mgmt.h"


double fixed_to_float(fixed_point_t input)
{
    return ((double)input / (double)(1 << FIXED_POINT_FRACTIONAL_BITS));
}

fixed_point_t float_to_fixed_euler(float input)
{
    return (fixed_point_t)(input * (1 << FIXED_POINT_FRACTIONAL_BITS_EULER));
    // return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

fixed_point_t float_to_fixed_quat(float input)
{
    return (fixed_point_t)(input * (1 << FIXED_POINT_FRACTIONAL_BITS_QUAT));
    // return (fixed_point_t)(round(input * (1 << FIXED_POINT_FRACTIONAL_BITS)));
}

void check_cpu_activity()
{
    nrf_gpio_pin_set(PIN_CPU_ACTIVITY);
}


/**@brief Handler for shutdown preparation.
 */
bool shutdown_handler(nrf_pwr_mgmt_evt_t event)
{
    uint32_t err_code;

    switch (event)
    {
        case NRF_PWR_MGMT_EVT_PREPARE_SYSOFF:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_SYSOFF");
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_WAKEUP:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_WAKEUP");
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_DFU:
            NRF_LOG_INFO("Power management wants to reset to DFU mode.");
            // APP_ERROR_HANDLER(NRF_ERROR_API_NOT_IMPLEMENTED);
            break;

        case NRF_PWR_MGMT_EVT_PREPARE_RESET:
            NRF_LOG_INFO("NRF_PWR_MGMT_EVT_PREPARE_RESET");
            break;
    }

    err_code = app_timer_stop_all();
    APP_ERROR_CHECK(err_code);

    return true;
}

/**@brief Register application shutdown handler with priority 0. */
NRF_PWR_MGMT_HANDLER_REGISTER(shutdown_handler, 0);

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


void app_scheduler_init(void)
{
    APP_SCHED_INIT(SCHED_MAX_EVENT_DATA_SIZE, SCHED_QUEUE_SIZE);
}

/**@brief Function for initializing low-frequency clock.
 */
void lfclk_config(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);

    nrf_drv_clock_lfclk_request(NULL);
}

void clocks_start(void)
{

    // Start LFXO and wait for it to start
    NRF_CLOCK->LFCLKSRC = CLOCK_LFCLKSRC_SRC_Xtal << CLOCK_LFCLKSRC_SRC_Pos;
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0);

    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);

    while(1);

}


void usr_set_poweroff_mode()
{
    int32_t ret = sd_power_system_off();

    NRF_POWER->TASKS_LOWPWR = 1;
    NRF_POWER->SYSTEMOFF = POWER_SYSTEMOFF_SYSTEMOFF_Enter;
    // while(1); // Check if this while is necessary
}