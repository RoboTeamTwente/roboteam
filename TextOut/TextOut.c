/*
 * TextOut.c
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#include "stm32f7xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "TextOut.h"
#include "usbd_cdc.h"
#include "usb_device.h"
#include "gpio_util.h"

uint8_t TxBuffer[1024];
char smallStrBuffer[1024];
extern USBD_HandleTypeDef hUsbDeviceFS;

void TextOut(char *str){
	int length = strlen(str);
	HexOut((uint8_t*)str, length);
}

void HexOut(uint8_t data[], uint8_t length){
	USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
//	set_pin(LD_3, LOW);
	if (hUsbDeviceFS.dev_state == 3 || hUsbDeviceFS.dev_state == 4) {
		while(hcdc->TxState != 0);
	//	set_pin(LD_3, HIGH);
		memcpy(TxBuffer, data, length);
		CDC_Transmit_FS(TxBuffer, length);
	}
}
