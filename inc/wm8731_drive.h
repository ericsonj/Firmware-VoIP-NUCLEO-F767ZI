/*
 * wm8731_drive.h
 *
 *  Created on: Jul 3, 2018
 *      Author: Ericson Joseph
 */

#ifndef WM8731_DRIVE_H_
#define WM8731_DRIVE_H_

/**************************************************************************************************
* WM8731 sound chip register addresses
**************************************************************************************************/
#define WM8731_ADDRESS				0x34		// WM8731 chip address on I2C bus
#define WM8731_REG_LLINE_IN			0x00		// Left Channel Line Input Volume Control
#define WM8731_REG_RLINE_IN 		0x01		// Right Channel Line Input Volume Control
#define WM8731_REG_LHPHONE_OUT 		0x02		// Left Channel Headphone Output Volume Control
#define WM8731_REG_RHPHONE_OUT 		0x03		// Right Channel Headphone Output Volume Control
#define WM8731_REG_ANALOG_PATH 		0x04		// Analog Audio Path Control
#define WM8731_REG_DIGITAL_PATH 	0x05		// Digital Audio Path Control
#define WM8731_REG_PDOWN_CTRL 		0x06		// Power Down Control Register
#define WM8731_REG_DIGITAL_IF 		0x07		// Digital Audio Interface Format
#define WM8731_REG_SAMPLING_CTRL 	0x08		// Sampling Control Register
#define WM8731_REG_ACTIVE_CTRL 		0x09		// Active Control
#define WM8731_REG_RESET 			0x0F		// Reset register

void WM8731_Init();

void WM8731_CMD(uint8_t address, uint16_t cmd);

#endif /* WM8731_DRIVE_H_ */
