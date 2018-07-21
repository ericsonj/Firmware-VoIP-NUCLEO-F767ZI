/*
 * wm8731_drive.c
 *
 *  Created on: Jul 3, 2018
 *      Author: Ericson Joseph
 */

#include <stdint.h>
#include "wm8731_drive.h"
#include "wm8731_imp.h"

void WM8731_Init(){
	HAL_init();
	HAL_delayms(200);
}

void WM8731_CMD(uint8_t address, uint16_t cmd) {
	uint8_t addr = address;
	addr = addr << 1;
	uint8_t cmd_upper = (uint8_t) (cmd >> 8);
	addr = addr | cmd_upper;
	uint8_t data[2] = { addr, (uint8_t) cmd };
	HAL_transmit(WM8731_ADDRESS, data, sizeof(data));
	HAL_delayms(20);
}

