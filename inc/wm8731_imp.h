/*
 * wm8731_imp.h
 *
 *  Created on: Jul 3, 2018
 *      Author: ejoseph
 */

#ifndef WM8731_IMP_H_
#define WM8731_IMP_H_

void HAL_init();

void HAL_transmit(uint8_t address, uint8_t *data, uint8_t dataLen);

void HAL_delayms(uint32_t time);

#endif /* WM8731_IMP_H_ */
