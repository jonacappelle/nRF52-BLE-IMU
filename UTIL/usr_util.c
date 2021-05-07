#include "usr_util.h"

#include "nrf_drv_gpiote.h"





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
    nrf_gpio_pin_set(CPU_ACTIVITY_PIN);
}
