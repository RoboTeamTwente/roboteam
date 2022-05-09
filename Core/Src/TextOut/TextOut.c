// /*
//  * TextOut.c
//  *
//  *  Created on: 14 sep. 2016
//  *      Author: Cas Doornkamp & Emiel Steerneman
//  */

#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "TextOut.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "gpio_util.h"
#include "REM_BaseTypes.h"
#include "REM_BasestationLog.h"

uint8_t TxBuffer[1024];
static char printf_buffer[1024];
extern USBD_HandleTypeDef hUsbDeviceFS;

// void LOG_printf(char *format, ...){
//     // if(!LOG_canAddLog()) return;

// 	// Place variables into string
// 	va_list aptr; 
// 	va_start(aptr, format); // Give starting point of additional arguments
//   vsnprintf(printf_buffer, 1024, format, aptr); // Copies and turns into string
//   va_end(aptr); // Close list
//   // Add the message to the buffer
//   LOG(printf_buffer);
// }

// /**
//  * @brief Sends a log message over USB using the PACKET_TYPE_REM_BASESTATION_LOG header.
//  * The log must always end with \n, which the function enforces.
//  * 
//  * TODO The entire message is now copied into another buffer, to prepend the 
//  * PACKET_TYPE_REM_BASESTATION_LOG header. This is inefficient. Is there a more efficient
//  * way or would that be premature optimization? How inefficient is this?
//  * 
//  * @param message The message to send over the USB
//  */
// // TODO Don't use malloc; Simply copy straight into the TxBuffer
// void LOG(char *message){
//   // Add +1 to the length of the string to account for the extra header bytes
// 	int length = strlen(message) + 1;
//   // Free up space for header + message
//   uint8_t* buffer = malloc(length);
//   // Set the header
//   buffer[0] = PACKET_TYPE_REM_BASESTATION_LOG;
//   // Enforce newline
//   buffer[length] = '\n';
//   // Copy the message into the buffer, next to the header
//   memcpy(buffer+1, message, length-1);
//   // Send the message over USB
// 	HexOut(buffer, length);
//   // Free up the memory
//   free(buffer);
// }

// void TextOut(char *str){
// 	int length = strlen(str);
// 	HexOut((uint8_t*)str, length);
// }

// void HexOut(uint8_t data[], uint8_t length){
// 	USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
// 	if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED || hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) {
// 		while(hcdc->TxState != 0);
// 		// TODO unneccesary memcpy? How about CDC_Transmit_FS(data, length);? 
// 		// 	 if CDC_Transmit_FS is blocking, no fear to have 'data' go out of scope.

//     REM_BasestationLog_set_header((REM_BasestationLogPayload*) data, PACKET_TYPE_REM_BASESTATION_LOG);  // 8 bits
//     REM_BasestationLog_set_remVersion((REM_BasestationLogPayload*) data, LOCAL_REM_VERSION);  // 4 bits
//     REM_BasestationLog_set_messageLength((REM_BasestationLogPayload*) data, 5); // 8 bits
//     sprintf(data+3, "56789\n");
//     // CDC_Transmit_FS(buffer, 8);

// 		memcpy(TxBuffer, data, 8);
// 		CDC_Transmit_FS(TxBuffer, 8);
// 	}
// }
