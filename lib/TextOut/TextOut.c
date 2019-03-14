/*
 * TextOut.c
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdio.h>
#include "TextOut.h"
#include "usbd_cdc.h"
#include "usb_device.h"

uint8_t TxBuffer[1024];
char smallStrBuffer[1024];
extern USBD_HandleTypeDef hUsbDeviceFS;

void TextOut(char *str){
	int length = strlen(str);
	HexOut((uint8_t*)str, length);
}

void HexOut(uint8_t data[], uint8_t length){
	USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
	HAL_GPIO_WritePin(LD0_GPIO_Port,LD0_Pin, 0);
	while(hcdc->TxState != 0);
	HAL_GPIO_WritePin(LD0_GPIO_Port,LD0_Pin, 1);
	memcpy(TxBuffer, data, length);
	CDC_Transmit_FS(TxBuffer, length);
}
