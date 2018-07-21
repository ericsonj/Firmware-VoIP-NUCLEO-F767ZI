/*
 * ej_stm32_i2c.c
 *
 *  Created on: Jul 4, 2018
 *      Author: ejoseph
 */
#include "ej_stm32_i2c.h"

static void _Error_Handler(char *file, int line);

void EJ_I2C_Init(I2C_HandleTypeDef *hi2c1) {

	__HAL_RCC_GPIOB_CLK_ENABLE()
	;

	hi2c1->Instance = I2C1;
	hi2c1->Init.Timing = 0x400005C0; //16 KHz
	hi2c1->Init.OwnAddress1 = 0;
	hi2c1->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1->Init.OwnAddress2 = 0;
	hi2c1->Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	if (HAL_I2C_Init(hi2c1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_I2CEx_ConfigAnalogFilter(hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	if (HAL_I2CEx_ConfigDigitalFilter(hi2c1, 0) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}
}

void HAL_I2C_MspInit(I2C_HandleTypeDef *hi2c) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hi2c->Instance == I2C1) {

		/* USER CODE BEGIN I2C1_MspInit 0 */

		/* USER CODE END I2C1_MspInit 0 */

		/**I2C1 GPIO Configuration
		 PB8     ------> I2C1_SCL
		 PB9     ------> I2C1_SDA
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral clock enable */
		__HAL_RCC_I2C1_CLK_ENABLE()
		;
		/* USER CODE BEGIN I2C1_MspInit 1 */

		/* USER CODE END I2C1_MspInit 1 */
	}

}

static void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

