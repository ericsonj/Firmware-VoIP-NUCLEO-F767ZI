/*
 * ej_gpiolib.c
 *
 *  Created on: Oct 6, 2018
 *      Author: ejoseph
 */

#include "main.h"
#include "ej_gpiolib.h"
#include "stm32f7xx_hal.h"

button_t callButton;

void EJ_GPIO_Init(void) {

	callButton.Pin = CALL_BUTTON_Pin;
	callButton.state = BUTTON_UP;
	callButton.tick = 0;

	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOG_CLK_ENABLE()
	;

	/*Configure GPIO pin : CALL_BUTTON_Pin */
	GPIO_InitStruct.Pin = CALL_BUTTON_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(CALL_BUTTON_GPIO_Port, &GPIO_InitStruct);

	HAL_NVIC_SetPriority(EXTI1_IRQn, 15, 0);
	HAL_NVIC_EnableIRQ(EXTI1_IRQn);

}

button_t *EJ_GPIO_GetButton(void){
	return &callButton;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == callButton.Pin) {
		GPIO_PinState val = HAL_GPIO_ReadPin(GPIOG, callButton.Pin);
		if (val == GPIO_PIN_SET) {
			callButton.state = BUTTON_UP;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
		} else {
			callButton.state = BUTTON_DOWN;
			HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
		}
	}
}

