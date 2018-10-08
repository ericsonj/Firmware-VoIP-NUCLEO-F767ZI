/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "main.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_nucleo_144.h"
#include <string.h>
#include <stdbool.h>
#include "wm8731_drive.h"
#include "wm8731_conf.h"
#include "stm32_adafruit_sd.h"
#include "ff.h"
#include "circbuffer.h"
#include "rtp.h"
#include "private.h"
#include "ej_gpiolib.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
#define SIZE 192
#define FRAME_SIZE 160
#define AUDIO_BUFFER_SIZE 10
#define FILENAME "audio.raw"
#define FILENAME_MIC "mic.raw"

typedef enum {
	WAIT_DATA, WAIT_TYPE, WAIT_LEN, READ_BYTES, TRANSMIT
} rxState_t;

SPI_HandleTypeDef hspi3;
DMA_HandleTypeDef hdma_spi3_tx;
DMA_HandleTypeDef hdma_spi3_rx;

SPI_HandleTypeDef hspi4;
DMA_HandleTypeDef hdma_spi4_rx;
DMA_HandleTypeDef hdma_spi4_tx;

uint16_t bufferTx[12];
uint16_t bufferRx[12];

SHORT audioFrame[FRAME_SIZE];
SHORT audioFrameMic[FRAME_SIZE];

static void loadBuffer(SPI_HandleTypeDef *hspi);
static void readBuffer(SPI_HandleTypeDef *hspi);

uint8_t UDPbufferTx[SIZE];
uint8_t UDPbufferRx[SIZE];
uint8_t DEBUGFrameRx[SIZE];
uint8_t DEBUGFrameRx2[SIZE];
uint8_t count = 1;
bool hasRTPDataTx = false;
bool hasRTPDataRx = false;

circ_buffer_t circ_buffer;
circ_buffer_t circ_buffer_mic;

typedef struct {
	SHORT audioFrame[FRAME_SIZE];
} frame_t;

frame_t AudioBuffer[120];
frame_t AUdioBufferMic[AUDIO_BUFFER_SIZE];

uint32_t frameCount = 1;
uint32_t icount = 1;
uint32_t frameIndex = 0;
uint8_t bufferId = 0;
bool bufferReady = false;
bool endFrame = false;

bool sendRTCP = true;

//static uint32_t ms_ticks; /**< 1ms timeticks counter */
static FATFS fs; /**< FatFs work area needed for each volume */
static FIL fp; /**< File object needed for each open file */
static FIL fp_mic; /**< File object needed for each open file */

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI3_Init(void);
static void MX_SPI4_Init(void);

// BUFFERS TX/RX RTP
uint8_t rtpFrameTx[172];
uint8_t rtpFrameRx[172];
uint8_t rtpFrameRxIdx = 0;

rxState_t rxState = WAIT_DATA;
uint8_t rxLenCount = 0;

uint8_t lagCount = 0;
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

DWORD get_fattime(void) {
	return ((DWORD) (2018 - 1980) << 25 | (DWORD) 06 << 21 | (DWORD) 24 << 16);
}

void WM8731_Activate() {
	WM8731_CMD(WM8731_REG_ACTIVE_CTRL, _WM8731_Activate);
}

void WM8731_Deactivate() {
	WM8731_CMD(WM8731_REG_ACTIVE_CTRL, _WM8731_Deactivate);
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 *
 * @retval None
 */

static void frameToAlaw(BYTE *out, SHORT *pcm, uint32_t len) {
	for (int i = 0; i < len; i++) {
		out[i] = (BYTE) linear2alaw(pcm[i]);
	}
}

static void alawtoFrame(SHORT *pcm, BYTE *in, uint32_t len) {
	for (int i = 0; i < len; i++) {
		pcm[i] = (SHORT) alaw2linear(in[i]);
	}
}

int main(void) {
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/
	bzero(bufferRx, sizeof(bufferRx));
	bzero(bufferTx, sizeof(bufferTx));
	bzero(audioFrame, sizeof(audioFrame));
	bzero(audioFrameMic, sizeof(audioFrameMic));

	bufferTx[11] = 0x8001;

	CIRC_BUFFER_InitRecidual(&circ_buffer, AudioBuffer, sizeof(AudioBuffer[0]),
			120, 110);

	CIRC_BUFFER_Init(&circ_buffer_mic, AUdioBufferMic,
			sizeof(AUdioBufferMic[0]), AUDIO_BUFFER_SIZE);

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	BSP_LED_Init(LED_RED);

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	EJ_GPIO_Init();

	HAL_Delay(100);

	WM8731_Init();

	HAL_Delay(100);

	WM8731_CMD(WM8731_REG_RESET, _WM8731_Reset);               	// Reset module
	WM8731_CMD(WM8731_REG_LLINE_IN, _WM8731_left_lineIn); // Left line in settings
	WM8731_CMD(WM8731_REG_RLINE_IN, _WM8731_Right_lineIn); // Rigth line in settings
	WM8731_CMD(WM8731_REG_LHPHONE_OUT, _WM8731_Left_hp); // Left headphone out settings
	WM8731_CMD(WM8731_REG_RHPHONE_OUT, _WM8731_Right_hp); // Right headphone out settings
	WM8731_CMD(WM8731_REG_ANALOG_PATH, _WM8731_AnalogAudio);	// Analog paths
	WM8731_CMD(WM8731_REG_DIGITAL_PATH, _WM8731_DigitalAudio);	// Digital paths
	WM8731_CMD(WM8731_REG_PDOWN_CTRL, _WM8731_power);	// Power down control
	WM8731_CMD(WM8731_REG_DIGITAL_IF, _WM8731_DAIF);		// Digital interface
	WM8731_CMD(WM8731_REG_SAMPLING_CTRL, _WM8731_Sampling);	// Sampling control

	WM8731_Activate();

	MX_DMA_Init();
	MX_SPI3_Init();
	MX_SPI4_Init();
	RTP_Init();

	/* USER CODE BEGIN 2 */
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	/* USER CODE END WHILE */

	frame_t frame;
	frame_t frameMic;

	//uint8_t rtpFrameRx[172];
	RTP_AddHeader(rtpFrameTx);

	BYTE alaw[FRAME_SIZE];
	BYTE alawRx[FRAME_SIZE];

	while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) == GPIO_PIN_RESET) {
	}
	HAL_SPI_TransmitReceive_DMA(&hspi4, UDPbufferTx, UDPbufferRx, SIZE);

	while (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4) == GPIO_PIN_RESET) {
	}
	HAL_SPI_TransmitReceive_DMA(&hspi3, (uint8_t*) bufferTx,
			(uint8_t*) bufferRx, sizeof(bufferTx) / 2);

//	if (f_mount(&fs, "", 0) != FR_OK) {
//	}

//	if (f_open(&fp, FILENAME, FA_READ | FA_WRITE | FA_CREATE_ALWAYS) == FR_OK) {

	button_t *callButton = EJ_GPIO_GetButton();

	BSP_LED_Init(LED_GREEN);
	BSP_LED_Init(LED_RED);
	BSP_LED_On(LED_GREEN);

	uint32_t ticksKeepAlive = 0;
	uint32_t nowTicks = 0;

	for (;;) {

		int32_t err = CIRC_BUFFER_pop(&circ_buffer_mic, &frameMic);
		if (err == ACTION_BUFFER_OK && callButton->state == BUTTON_DOWN) {
			frameToAlaw(alaw, frameMic.audioFrame, FRAME_SIZE);
			RTP_AddVarHeader(rtpFrameTx);
			memcpy(rtpFrameTx + 12, alaw, FRAME_SIZE);
			hasRTPDataTx = true;
//			i++;
		}
		if (hasRTPDataRx) {
			if (rtpFrameRx[0] == 0x80) {
				memcpy(alawRx, rtpFrameRx + 12, FRAME_SIZE);
//				f_write(&fp, alawRx, FRAME_SIZE, &num_write);
				alawtoFrame(frame.audioFrame, alawRx, FRAME_SIZE);
				CIRC_BUFFER_push(&circ_buffer, &frame);
			}
			hasRTPDataRx = false;
		}
//		else {
		nowTicks = HAL_GetTick();
		if ((nowTicks - ticksKeepAlive) > 20000) {
			ticksKeepAlive = nowTicks;
			sendRTCP = true;
		}
//		}

	}

//		f_close(&fp);

//	}

	while (1) {
	}

}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 96;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 2;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* SPI3 init function */
static void MX_SPI3_Init(void) {

	/* SPI3 parameter configuration*/
	hspi3.Instance = SPI3;
	hspi3.Init.Mode = SPI_MODE_SLAVE;
	hspi3.Init.Direction = SPI_DIRECTION_2LINES;
	hspi3.Init.DataSize = SPI_DATASIZE_16BIT;
	hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi3.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi3.Init.CRCPolynomial = 7;
	hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
//	hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLED;
	if (HAL_SPI_Init(&hspi3) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* SPI4 init function */
static void MX_SPI4_Init(void) {

	/* SPI4 parameter configuration*/
	hspi4.Instance = SPI4;
	hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
	hspi4.Init.Mode = SPI_MODE_SLAVE;
	hspi4.Init.Direction = SPI_DIRECTION_2LINES;
	hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi4.Init.NSS = SPI_NSS_HARD_INPUT;
	hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi4.Init.CRCPolynomial = 7;
	hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi4) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/**
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE()
	;
	/* DMA interrupt init */
	/* DMA1_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
	/* DMA1_Stream5_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

	/* DMA controller clock enable */
	__HAL_RCC_DMA2_CLK_ENABLE()
	;
	/* DMA interrupt init */
	/* DMA2_Stream0_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
	/* DMA2_Stream1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 4, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 PC1   ------> ETH_MDC
 PA1   ------> ETH_REF_CLK
 PA2   ------> ETH_MDIO
 PA7   ------> ETH_CRS_DV
 PC4   ------> ETH_RXD0
 PC5   ------> ETH_RXD1
 PB13   ------> ETH_TXD1
 PD8   ------> USART3_TX
 PD9   ------> USART3_RX
 PA8   ------> USB_OTG_FS_SOF
 PA9   ------> USB_OTG_FS_VBUS
 PA10   ------> USB_OTG_FS_ID
 PA11   ------> USB_OTG_FS_DM
 PA12   ------> USB_OTG_FS_DP
 PG11   ------> ETH_TX_EN
 PG13   ------> ETH_TXD0
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14 | GPIO_PIN_7, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PC1 PC4 PC5 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_4 | GPIO_PIN_5;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : PA1 PA2 PA7 */
	GPIO_InitStruct.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : PB14 PB7 */
	GPIO_InitStruct.Pin = GPIO_PIN_14 | GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	/*Configure GPIO pins : PD8 PD9 */
//	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
//	GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
//	HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	/*Configure GPIO pin : PG6 */
	GPIO_InitStruct.Pin = GPIO_PIN_6;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pin : PG7 */
	GPIO_InitStruct.Pin = GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PA8 PA10 PA11 PA12 */
	GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PA9 */
	GPIO_InitStruct.Pin = GPIO_PIN_9;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PG11 PG13 */
	GPIO_InitStruct.Pin = GPIO_PIN_11 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF11_ETH;
	HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

	/*Configure GPIO pins : PF12 */
	GPIO_InitTypeDef GPIO_InitStruct2;
	GPIO_InitStruct2.Pin = GPIO_PIN_12;
	GPIO_InitStruct2.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct2.Pull = GPIO_PULLUP;
	GPIO_InitStruct2.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	HAL_GPIO_Init(GPIOF, &GPIO_InitStruct2);

}

static void readBuffer(SPI_HandleTypeDef *hspi) {
	uint16_t offset = (SIZE - (uint8_t) hspi->hdmarx->Instance->NDTR);
	memcpy(DEBUGFrameRx, UDPbufferRx, SIZE); // KEY FOR BETTER
	for (int i = offset; i < SIZE; i++) {
		switch (rxState) {
		case WAIT_DATA:
			if (DEBUGFrameRx[offset % SIZE] == 0x84) {
				rxState = WAIT_TYPE;
			}
			break;
		case WAIT_TYPE:
			if (DEBUGFrameRx[offset % SIZE] == 0x00) {
				rxState = WAIT_LEN;
			} else {
				rxState = WAIT_DATA;
			}
			break;
		case WAIT_LEN:
			rxLenCount = DEBUGFrameRx[offset % SIZE];
			if (rxLenCount == 0) {
				rxState = WAIT_DATA;
			} else {
				rtpFrameRxIdx = 0;
				rxState = READ_BYTES;
			}
			break;
		case READ_BYTES:
			rtpFrameRx[rtpFrameRxIdx] = DEBUGFrameRx[offset % SIZE];
			rtpFrameRxIdx++;
			rxLenCount--;
			if (rxLenCount == 0) {
				rxState = TRANSMIT;
			}
			break;
		case TRANSMIT:
			hasRTPDataRx = true;
			rxState = WAIT_DATA;
			break;
		}
		offset++;
	}
}

static void loadBuffer(SPI_HandleTypeDef *hspi) {

	uint16_t offset = (SIZE - (uint8_t) hspi->hdmatx->Instance->NDTR) + 1;
	UDPbufferTx[offset++] = 0x84; // init data
	UDPbufferTx[offset++] = 0x00; // type
	UDPbufferTx[offset++] = 0xAC; // 172 len
	uint16_t i = 0;
	while (i < 172) {
		UDPbufferTx[offset % SIZE] = rtpFrameTx[i];
		offset++;
		i++;
	}
}

static void loadRTCP(SPI_HandleTypeDef *hspi) {

	char *cname = "1005@ericsonj.net";
	uint8_t cnameLen = strlen(cname);
	uint8_t dataLen = cnameLen + 10;
	uint16_t rtcplen = (dataLen / 4) + ((dataLen % 4) > 0 ? 1 : 0) - 1;

	uint16_t offset = (SIZE - (uint8_t) hspi->hdmatx->Instance->NDTR) + 1;
	UDPbufferTx[offset++] = 0x84; // init data
	UDPbufferTx[offset++] = 0x00; // type
	UDPbufferTx[offset++] = dataLen; // 172 len
	UDPbufferTx[offset++] = 0x81; // RTCP
	UDPbufferTx[offset++] = 0xCA; // SDES

	uint16_t hight = rtcplen & 0xFF00;
	hight >>= 8;
	UDPbufferTx[offset++] = (uint8_t) hight;
	uint8_t low = rtcplen & 0x000FF;
	UDPbufferTx[offset++] = low;

	uint32_t ssrc = 16384;
	uint8_t b;
	b = ssrc & 0xFF000000;
	b >>= 24;
	UDPbufferTx[offset++] = b;

	b = ssrc & 0x00FF0000;
	b >>= 16;
	UDPbufferTx[offset++] = b;

	b = ssrc & 0x0000FF00;
	b >>= 8;
	UDPbufferTx[offset++] = b;

	b = ssrc & 0x000000FF;
	UDPbufferTx[offset++] = b;

	UDPbufferTx[offset++] = 0x01;
	UDPbufferTx[offset++] = cnameLen;

	for (int i = 0; i < cnameLen; ++i) {
		UDPbufferTx[offset++] = cname[i];
	}

}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {

	if (hspi->Instance == SPI3) {

		if (frameIndex >= 160) {

			frame_t frame;
			frame_t frameMic;

			int32_t err;
			err = CIRC_BUFFER_pop(&circ_buffer, &frame);
			if (err == ACTION_BUFFER_OK) {
				memcpy(audioFrame, frame.audioFrame, 320);
//				BSP_LED_Off(LED_RED);
			} else {
				bzero(audioFrame, sizeof(audioFrame));
//				BSP_LED_On(LED_RED);
			}

			bzero(frameMic.audioFrame, sizeof(frameMic.audioFrame));
			if (CIRC_BUFFER_hasSpace(&circ_buffer_mic)) {
				memcpy(frameMic.audioFrame, audioFrameMic, 320);
				CIRC_BUFFER_push(&circ_buffer_mic, &frameMic);
			}
			frameIndex = 0;

		}

		if (icount % 2 == 0) {
			bufferTx[11] = audioFrame[frameIndex];
			audioFrameMic[frameIndex] = bufferRx[11] * 50;
			frameIndex++;
		}
		icount++;

	} else if (hspi->Instance == SPI4) {
		memset(UDPbufferTx, 0, SIZE);
		if (hasRTPDataTx) {
			loadBuffer(hspi);
			hasRTPDataTx = false;
		} else if (sendRTCP) {
			loadRTCP(hspi);
			sendRTCP = false;
		}
		readBuffer(hspi);
	}

}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == SPI4) {
		while (HAL_GPIO_ReadPin(GPIOE, GPIO_PIN_11) != GPIO_PIN_SET) // CS signal
		{
		}
	}
}

void HAL_SPI_TxRxHalfCpltCallback(SPI_HandleTypeDef *hspi) {
	HAL_GPIO_DeInit(GPIOA, GPIO_PIN_4);
//	GPIOF->ODR ^= GPIO_PIN_12;
}

//void EXTI0_IRQHandler(void) {
//	if ((EXTI->PR & 0x0001) == 0x0001) {
////		GPIOF->ODR ^= GPIO_PIN_12;
//		EXTI->PR = 0x0001;
//	}
//}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
