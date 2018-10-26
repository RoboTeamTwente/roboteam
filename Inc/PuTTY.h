/*
Author: Antonio
Date: 26/10/2018

Description: 

Instructions:

Extra functions:

GPIO Pins:

Notes:
 */

#include <stdbool.h> // Bools
#include <string.h> // Strings
#include <stdio.h> // General

// Commands to be rememberd
#define COMMANDS_TO_REMEMBER 16
#define MAX_COMMAND_LENGTH   32

// TODO: USART or USB? It seems USB is never used tbh.

// Callback function (to place after HAL_UART_RxCpltCallback)
void puttyCallback(PuttyInterfaceTypeDef puttyIn, UART_HandleTypeDef *huart){
    puttyIn.huart_Rx_len = 1;
    puttyIn.small_buf[0] = *(huart->pRxBuffPtr-1);
}