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