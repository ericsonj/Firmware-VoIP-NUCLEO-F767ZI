/*
 * ej_gpiolib.h
 *
 *  Created on: Oct 6, 2018
 *      Author: ejoseph
 */

#ifndef EJ_GPIOLIB_H_
#define EJ_GPIOLIB_H_

#include <stdint.h>

typedef enum {
	BUTTON_UP, BUTTON_FALLING, BUTTON_DOWN, BUTTON_RASING
} buttonState_t;

typedef struct {
	uint32_t Pin;
	buttonState_t state;
	uint32_t tick;
} button_t;

void EJ_GPIO_Init(void);

button_t *EJ_GPIO_GetButton(void);

#endif /* EJ_GPIOLIB_H_ */
