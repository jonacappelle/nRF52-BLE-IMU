#ifndef _USR_UTIL_H_
#define _USR_UTIL_H_

#include "stdio.h"
#include <stdint.h>

#include "app_timer.h"


#define PIN_CPU_ACTIVITY    18
#define PIN_IMU_ACTIVITY    20


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

// Convert frequency to milliseconds
#define FREQ_TO_MS(x) ((1.000/x)*1000)


void idle_state_handle(void);


#define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */

void app_scheduler_init(void);

void lfclk_config(void);
void clocks_start(void);

#endif
