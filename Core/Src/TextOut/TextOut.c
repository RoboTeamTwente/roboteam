/*
 * TextOut.c
 *
 *  Created on: 14 sep. 2016
 *      Author: Cas Doornkamp & Emiel Steerneman
 */

#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "TextOut.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "gpio_util.h"
#include "REM_BaseTypes.h"

uint8_t TxBuffer[1024];
extern USBD_HandleTypeDef hUsbDeviceFS;

/**
 * @brief Sends a log message over USB using the PACKET_TYPE_REM_BASESTATION_LOG header.
 * The log must always end with \n, which the function enforces.
 * 
 * TODO The entire message is now copied into another buffer, to prepend the 
 * PACKET_TYPE_REM_BASESTATION_LOG header. This is inefficient. Is there a more efficient
 * way or would that be premature optimization? How inefficient is this?
 * 
 * @param message The message to send over the USB
 */
// TODO Don't use malloc; Simply copy straight into the TxBuffer
void LOG(char *message){
  // Add +1 to the length of the string to account for the extra header bytes
	int length = strlen(message) + 1;
  // Free up space for header + message
  uint8_t* buffer = malloc(length);
  // Set the header
  buffer[0] = PACKET_TYPE_REM_BASESTATION_LOG;
  // Enforce newline
  buffer[length] = '\n';
  // Copy the message into the buffer, next to the header
  memcpy(buffer+1, message, length-1);
  // Send the message over USB
	HexOut(buffer, length);
  // Free up the memory
  free(buffer);
}

void TextOut(char *str){
	int length = strlen(str);
	HexOut((uint8_t*)str, length);
}

void HexOut(uint8_t data[], uint8_t length){
	USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	if (hUsbDeviceFS.dev_state == 3 || hUsbDeviceFS.dev_state == 4) {
		while(hcdc->TxState != 0);
		// TODO unneccesary memcpy? How about CDC_Transmit_FS(data, length);? 
		// 	 if CDC_Transmit_FS is blocking, no fear to have 'data' go out of scope.
		memcpy(TxBuffer, data, length);
		CDC_Transmit_FS(TxBuffer, length);
	}
}
