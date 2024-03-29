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
 *         File: usr_gpio.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: GPIO Header file
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#ifndef _USR_GPIO_H_
#define _USR_GPIO_H_

// GPIO Interrupt
#include <stdbool.h>
#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "app_error.h"
#include "boards.h"
#include "nrf_delay.h"

// Initialization of GPIO drv
void gpio_init(void);
void gpiote_evt_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action);

// On-board LED functionality
void led_init();
void led_deinit();
void led_on();
void led_off();
void led_toggle();
void led_flash2();

// LED Softblink (PWM based led control)
void LED_softblink_start();
void led_chg_start();
void led_chg_stop();

#endif
