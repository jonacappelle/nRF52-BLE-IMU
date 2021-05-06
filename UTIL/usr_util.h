#ifndef _USR_UTIL_H_
#define _USR_UTIL_H_

#include "stdio.h"
#include <stdint.h>


/// Fixed-point Format: 11.5 (16-bit)
typedef int32_t fixed_point_t;

#define FIXED_POINT_FRACTIONAL_BITS_EULER       16
#define FIXED_POINT_FRACTIONAL_BITS_QUAT        30
#define FIXED_POINT_FRACTIONAL_BITS             0

double fixed_to_float(fixed_point_t input);
fixed_point_t float_to_fixed_euler(float input);
fixed_point_t float_to_fixed_quat(float input);

void check_cpu_activity();


#define FRACT_BITS 16
#define FRACT_BITS_D2 8
#define FIXED_ONE (1 << FRACT_BITS)
#define INT2FIXED(x) ((x) << FRACT_BITS)
#define FLOAT2FIXED(x) ((int)((x) * (1 << FRACT_BITS))) 
#define FIXED2INT(x) ((x) >> FRACT_BITS)
#define FIXED2DOUBLE(x) (((double)(x)) / (1 << FRACT_BITS))
#define MULT(x, y) ( ((x) >> FRACT_BITS_D2) * ((y)>> FRACT_BITS_D2) )


#endif
