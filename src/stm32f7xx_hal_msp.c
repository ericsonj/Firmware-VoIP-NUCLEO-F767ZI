/**
 ******************************************************************************
 * File Name          : stm32f7xx_hal_msp.c
 * Description        : This file provides code for the MSP Initialization
 *                      and de-Initialization codes.
 ******************************************************************************
 ** This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * COPYRIGHT(c) 2018 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "main.h"

extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;

extern DMA_HandleTypeDef hdma_spi4_rx;
extern DMA_HandleTypeDef hdma_spi4_tx;

extern void _Error_Handler(char *, int);
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
/**
 * Initializes the Global MSP.
 */
void HAL_MspInit(void) {
	/* USER CODE BEGIN MspInit 0 */

	/* USER CODE END MspInit 0 */

	HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4);

	/* System interrupt init*/
	/* MemoryManagement_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(MemoryManagement_IRQn, 0, 0);
	/* BusFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(BusFault_IRQn, 0, 0);
	/* UsageFault_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(UsageFault_IRQn, 0, 0);
	/* SVCall_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SVCall_IRQn, 0, 0);
	/* DebugMonitor_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DebugMonitor_IRQn, 0, 0);
	/* PendSV_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

	/* USER CODE BEGIN MspInit 1 */

	/* USER CODE END MspInit 1 */
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* hspi) {

	GPIO_InitTypeDef GPIO_InitStruct;
	if (hspi->Instance == SPI3) {
		/* USER CODE BEGIN SPI3_MspInit 0 */

		/* USER CODE END SPI3_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI3_CLK_ENABLE()
		;

		/**SPI3 GPIO Configuration
		 PA4     ------> SPI3_NSS
		 PC10     ------> SPI3_SCK
		 PC11     ------> SPI3_MISO
		 PC12     ------> SPI3_MOSI
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_4;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN; // <<<
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_10;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_PULLDOWN;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_12;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_11;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; // <<<
		GPIO_InitStruct.Pull = GPIO_PULLDOWN; // <<<
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* SPI3 DMA Init */
		/* SPI3_TX Init */
		hdma_spi3_tx.Instance = DMA1_Stream5;
		hdma_spi3_tx.Init.Channel = DMA_CHANNEL_0;
		hdma_spi3_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi3_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi3_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi3_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_spi3_tx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_spi3_tx.Init.Mode = DMA_CIRCULAR;
		hdma_spi3_tx.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_spi3_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi3_tx) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(hspi, hdmatx, hdma_spi3_tx);

		/* SPI3_RX Init */
		hdma_spi3_rx.Instance = DMA1_Stream0;
		hdma_spi3_rx.Init.Channel = DMA_CHANNEL_0;
		hdma_spi3_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_spi3_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi3_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi3_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_HALFWORD;
		hdma_spi3_rx.Init.MemDataAlignment = DMA_MDATAALIGN_HALFWORD;
		hdma_spi3_rx.Init.Mode = DMA_CIRCULAR;
		hdma_spi3_rx.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_spi3_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		if (HAL_DMA_Init(&hdma_spi3_rx) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(hspi, hdmarx, hdma_spi3_rx);

		/* SPI3 interrupt Init */
		HAL_NVIC_SetPriority(SPI3_IRQn, 0, 0);
		HAL_NVIC_EnableIRQ(SPI3_IRQn);

		/* USER CODE BEGIN SPI3_MspInit 1 */

		/* USER CODE END SPI3_MspInit 1 */
	}

	if (hspi->Instance == SPI4) {
		/* USER CODE BEGIN SPI4_MspInit 0 */

		/* USER CODE END SPI4_MspInit 0 */
		/* Peripheral clock enable */
		__HAL_RCC_SPI4_CLK_ENABLE()
		;

		/**SPI4 GPIO Configuration
		 PE11     ------> SPI4_NSS
		 PE12     ------> SPI4_SCK
		 PE13     ------> SPI4_MISO
		 PE14     ------> SPI4_MOSI
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13
				| GPIO_PIN_14;
		GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
		GPIO_InitStruct.Alternate = GPIO_AF5_SPI4;
		HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

		/* SPI4 DMA Init */
		/* SPI4_RX Init */
		hdma_spi4_rx.Instance = DMA2_Stream0;
		hdma_spi4_rx.Init.Channel = DMA_CHANNEL_4;
		hdma_spi4_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_spi4_rx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi4_rx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi4_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi4_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi4_rx.Init.Mode = DMA_CIRCULAR;
		hdma_spi4_rx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_spi4_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_spi4_rx.Instance->NDTR = 64;
		if (HAL_DMA_Init(&hdma_spi4_rx) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(hspi, hdmarx, hdma_spi4_rx);

		/* SPI4_TX Init */
		hdma_spi4_tx.Instance = DMA2_Stream1;
		hdma_spi4_tx.Init.Channel = DMA_CHANNEL_4;
		hdma_spi4_tx.Init.Direction = DMA_MEMORY_TO_PERIPH;
		hdma_spi4_tx.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_spi4_tx.Init.MemInc = DMA_MINC_ENABLE;
		hdma_spi4_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
		hdma_spi4_tx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
		hdma_spi4_tx.Init.Mode = DMA_CIRCULAR;
		hdma_spi4_tx.Init.Priority = DMA_PRIORITY_VERY_HIGH;
		hdma_spi4_tx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_spi4_tx.Instance->NDTR = 64;
		if (HAL_DMA_Init(&hdma_spi4_tx) != HAL_OK) {
			_Error_Handler(__FILE__, __LINE__);
		}

		__HAL_LINKDMA(hspi, hdmatx, hdma_spi4_tx);

		/* USER CODE BEGIN SPI4_MspInit 1 */

		/* USER CODE END SPI4_MspInit 1 */
	}

}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* hspi) {

	if (hspi->Instance == SPI3) {
		/* USER CODE BEGIN SPI3_MspDeInit 0 */

		/* USER CODE END SPI3_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI3_CLK_DISABLE();

		/**SPI3 GPIO Configuration
		 PA4     ------> SPI3_NSS
		 PC10     ------> SPI3_SCK
		 PC11     ------> SPI3_MISO
		 PC12     ------> SPI3_MOSI
		 */
		HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);

		HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12);

		/* SPI3 DMA DeInit */
		HAL_DMA_DeInit(hspi->hdmatx);
		HAL_DMA_DeInit(hspi->hdmarx);

		/* SPI3 interrupt DeInit */
		HAL_NVIC_DisableIRQ(SPI3_IRQn);

		/* USER CODE BEGIN SPI3_MspDeInit 1 */

		/* USER CODE END SPI3_MspDeInit 1 */
	}

	if (hspi->Instance == SPI4) {
		/* USER CODE BEGIN SPI4_MspDeInit 0 */

		/* USER CODE END SPI4_MspDeInit 0 */
		/* Peripheral clock disable */
		__HAL_RCC_SPI4_CLK_DISABLE();

		/**SPI4 GPIO Configuration
		 PE11     ------> SPI4_NSS
		 PE12     ------> SPI4_SCK
		 PE13     ------> SPI4_MISO
		 PE14     ------> SPI4_MOSI
		 */
		HAL_GPIO_DeInit(GPIOE,
				GPIO_PIN_11 | GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14);

		/* SPI4 DMA DeInit */
		HAL_DMA_DeInit(hspi->hdmarx);
		HAL_DMA_DeInit(hspi->hdmatx);
		/* USER CODE BEGIN SPI4_MspDeInit 1 */

		/* USER CODE END SPI4_MspDeInit 1 */
	}

}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
