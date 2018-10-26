/*
 * PuttyInterface.h
 *
 *  Created on: Sep 29, 2017
 *      Author: Leon
 */

#ifndef PUTTYINTERFACE_H_
#define PUTTYINTERFACE_H_

#define PUTTY_USART

#ifdef PUTTY_USART
#define huartx huart3
#endif /* PUTTY_USART */

#ifdef PUTTY_USART
#include "usart.h"
#endif/* PUTTY_USART */
// #ifdef PUTTY_USB
// #include "usbd_cdc_if.h"
// #endif /* PUTTY_USB */
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// amount of commands that will be remembered
#define COMMANDS_TO_REMEMBER 16
#define MAX_COMMAND_LENGTH   32

// function that works like normal printf()
#define uprintf(...) { sprintf(smallStrBuffer, __VA_ARGS__); \
	TextOut(smallStrBuffer);}
//#define uprintf(...) {}

// function that will be called when HandlePcInput is done.
typedef void (*HandleLine)(char * input);
// Explanation: Basically, make a function for dealing with inputs and assign it to this struct.

typedef struct {
	uint8_t rec_buf[32];
	char small_buf[32];
	uint huart_Rx_len;
	HandleLine handle;
}PuttyInterfaceTypeDef;

char smallStrBuffer[1024]; // Used for the output

/* 	Print string str to the pc
 * 	param:
 * 		str: string to print
 */
void TextOut(char *str);

/*	Transmit raw data
 * 	param:
 * 		data[]: the data
 * 		length: the length of the data to send
 */
void HexOut(uint8_t data[], uint8_t length);

void PuttyInterface_Init(PuttyInterfaceTypeDef* pitd);

void PuttyInterface_Update(PuttyInterfaceTypeDef* pitd);

#endif /* PUTTYINTERFACE_H_ */
