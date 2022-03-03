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
 *         File: usr_flash.h
 *      Created: 2022-03-01
 *       Author: Jona Cappelle
 *      Version: 1.0
 *
 *  Description: Interaction with internal flash memory
 *
 *  Commissiond by Interreg NOMADe
 *
 */

#ifndef _USR_FLASH_H_
#define _USR_FLASH_H_

#include "fds.h"

#include <stdint.h>
#include <string.h>
#include "nordic_common.h"

void usr_flash_init();
void usr_flash_write(uint8_t const * data, const uint32_t len);
void usr_flash_read(uint8_t * data, uint32_t len);
bool usr_flash_check_valid_record();

#endif