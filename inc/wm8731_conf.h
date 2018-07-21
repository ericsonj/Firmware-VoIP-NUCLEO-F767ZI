/*
 * wm8731_conf.h
 *
 *  Created on: Jul 5, 2018
 *      Author: ejoseph
 */

#ifndef WM8731_CONF_H_
#define WM8731_CONF_H_

/**************************************************************************************************
 * WM8731 sound chip #define uint16_tants (for default set up)
 **************************************************************************************************/
#define _WM8731_left_lineIn 	0x0180		// Mic settings: Enable mute, Enable simultaneous load to left and right channels
#define _WM8731_Right_lineIn 	0x0180 		// Mic settings: Enable mute, Enable simultaneous load to left and right channels
#define _WM8731_Left_hp 		0x00F9		// Headphone settings : -9dB output
#define _WM8731_Right_hp 		0x00F9		// Headphone settings : -9dB output
//#define _WM8731_AnalogAudio 	0xD0;			// DAC Select
//#define _WM8731_AnalogAudio		0x24			// sideTone Select
#define _WM8731_AnalogAudio		0x14			// DAC Select
#define _WM8731_DigitalAudio 	0x06
#define _WM8731_power 			0x00			// Disable Power down
#define _WM8731_DAIF 			0x40			// Enable Master Mode and 16bit data
//#define _WM8731_DAIF 			0x53			// Enable Master Mode and DSP 16bit data
#define _WM8731_Sampling 		0x0C			// 8000Hz
#define _WM8731_Activate 		0x01			// Module is ON
#define _WM8731_Deactivate 		0x00			// Module is OFF
#define _WM8731_Reset 			0x00			// Reset value

#endif /* WM8731_CONF_H_ */
