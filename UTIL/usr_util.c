#include "usr_util.h"

#include "nrf_drv_gpiote.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

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