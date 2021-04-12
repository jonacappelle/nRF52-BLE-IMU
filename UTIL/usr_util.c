#include "usr_util.h"



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

// If round() is not supported (may lose some accuracy)
// inline fixed_point_t float_to_fixed(double input)
// {
//     return (fixed_point_t)(input * (1 << FIXED_POINT_FRACTIONAL_BITS));
// }

