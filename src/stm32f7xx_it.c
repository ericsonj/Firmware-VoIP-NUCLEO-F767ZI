/**
 ******************************************************************************
 * @file    stm32f7xx_it.c
 * @author  Ac6
 * @version V1.0
 * @date    02-Feb-2015
 * @brief   Default Interrupt Service Routines.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"
#include "stm32f7xx.h"
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include "stm32f7xx_it.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_spi3_tx;
extern DMA_HandleTypeDef hdma_spi3_rx;
extern SPI_HandleTypeDef hspi3;

extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
 * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
 * @param  None
 * @retval None
 */
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

/******************************************************************************/
/* STM32F7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f7xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

	/* USER CODE END DMA1_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_spi3_rx);
	/* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

	/* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream5 global interrupt.
 */
void DMA1_Stream5_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream5_IRQn 0 */

	/* USER CODE END DMA1_Stream5_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_spi3_tx);
	/* USER CODE BEGIN DMA1_Stream5_IRQn 1 */

	/* USER CODE END DMA1_Stream5_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream1 global interrupt.
 */
void DMA1_Stream1_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

	/* USER CODE END DMA1_Stream1_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_rx);
	/* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

	/* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream3 global interrupt.
 */
void DMA1_Stream3_IRQHandler(void) {
	/* USER CODE BEGIN DMA1_Stream3_IRQn 0 */

	/* USER CODE END DMA1_Stream3_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_usart3_tx);
	/* USER CODE BEGIN DMA1_Stream3_IRQn 1 */

	/* USER CODE END DMA1_Stream3_IRQn 1 */
}

/**
 * @brief This function handles SPI3 global interrupt.
 */
void SPI3_IRQHandler(void) {
	/* USER CODE BEGIN SPI3_IRQn 0 */
	/* USER CODE END SPI3_IRQn 0 */
	HAL_SPI_IRQHandler(&hspi3);
	/* USER CODE BEGIN SPI3_IRQn 1 */
//	HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_12);
	/* USER CODE END SPI3_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
