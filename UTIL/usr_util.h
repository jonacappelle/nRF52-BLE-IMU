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
 *         File: usr_util.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: Other utilities
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#ifndef _USR_UTIL_H_
#define _USR_UTIL_H_

#include "stdio.h"
#include <stdint.h>

#include "app_timer.h"
#include "ble_motion_service.h"


#define PIN_CPU_ACTIVITY    18
#define PIN_IMU_ACTIVITY    20


/// Fixed-point Format: 11.5 (16-bit)
typedef int32_t fixed_point_t;

#define FIXED_POINT_FRACTIONAL_BITS_EULER       16
#define FIXED_POINT_FRACTIONAL_BITS_QUAT        30
#define FIXED_POINT_FRACTIONAL_BITS             0

// Efficient data transmission over BLE
double fixed_to_float(fixed_point_t input);
fixed_point_t float_to_fixed_euler(float input);
fixed_point_t float_to_fixed_quat(float input);

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

// Sleep
void idle_state_handle(void);
// Deep Sleep
void usr_set_poweroff_mode();

// Application scheduler
// #define SCHED_MAX_EVENT_DATA_SIZE   APP_TIMER_SCHED_EVENT_DATA_SIZE /**< Maximum size of scheduler events. */
#define SCHED_MAX_EVENT_DATA_SIZE   sizeof(ble_tms_config_t) /**< Maximum size of scheduler events. */
#define SCHED_QUEUE_SIZE            60  /**< Maximum number of events in the scheduler queue. */
void app_scheduler_init(void);

// Initial clock config
void lfclk_config(void);
void clocks_start(void);

// Debugging - check what caused a reset
void check_reset_reason();
// CPU activity is put through on a GPIO pin
void check_cpu_activity();

// Logging over Segger RTT
void log_init(void);

// Stack guard: eliminate weird problems
void stack_guard_init();

#endif
