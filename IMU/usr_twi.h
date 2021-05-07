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
 *         File: usr_twi.h
 *      Created: YYYY-MM-DD
 *       Author: Jona Cappelle
 *      Version: v1.0
 *
 *  Description: I2C communication header file
 *
 *  Commissiond by Interreg NOMADe
 * 
 */

#ifndef _USR_TWI_H_
#define _USR_TWI_H_

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"


/* TWI instance ID. */
#define TWI_INSTANCE_ID     0

uint32_t twi_open (void);
uint32_t twi_close(void);

void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
ret_code_t i2c_write_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, const uint8_t* data, uint32_t len, bool stop);
ret_code_t i2c_write_forread_byte(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address);
ret_code_t i2c_read_bytes(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);
ret_code_t i2c_read_bytes_dma(const nrf_drv_twi_t *twi_handle, uint8_t address, uint8_t sub_address, uint8_t * dest, uint8_t dest_count);


#endif
