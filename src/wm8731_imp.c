/*
 * wm8731_imp.c
 *
 *  Created on: Jul 3, 2018
 *      Author: Ericson Joseph
 */

#include <stdint.h>
#include "wm8731_imp.h"
#include "ej_stm32_i2c.h"

static I2C_HandleTypeDef hi2c1;

void HAL_init() {
	EJ_I2C_Init(&hi2c1);
}

void HAL_transmit(uint8_t address, uint8_t *data, uint8_t dataLen) {
	HAL_I2C_Master_Transmit(&hi2c1, address, data, dataLen, 1);
}

void HAL_delayms(uint32_t time) {
	HAL_Delay(time);
}
