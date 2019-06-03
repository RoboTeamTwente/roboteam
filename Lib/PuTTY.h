/*
 * PuTTY.h
 *
 *  Created on: 9 apr. 2019
 *      Author: cjdoo
 */

#ifndef PUTTY_H_
#define PUTTY_H_

#include "stdint.h" // Includes the uint8_t and others
#include <stdarg.h>  // for formatting in putty_printf

#include "../Core/Inc/dribbler.h"
#include "../Core/Inc/geneva.h"
#include "../Core/Inc/shoot.h"
#include "../Core/Inc/wheels.h"
#include "../Core/Inc/testFunctions.h"

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs
typedef enum
{
    NoErrors = 0
} Putty_Enum;

typedef struct
{
    uint8_t rec_buf[32];       // Buffer with received data
    char small_buf[32];        // Buffer for use in HandlePCInput
    char smallStrBuffer[1024]; // Used in the Putty_Printf
    uint8_t TxBuf[1024];       // Transimission buffer
    unsigned int huart_Rx_len;         // Length of received data
    Putty_Enum errorCode;      // Error codes
} Putty_VarStruct;

Putty_VarStruct Putty_Vars; // Global struct with all variables

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC
Putty_Enum Putty_Init();              // Also returns a pointer to the variable struct
Putty_Enum Putty_GetErrorCode();      // Returns the error code
Putty_Enum Putty_printf(char* format, ...); // Like printf, but for the putty console
Putty_Enum Putty_DeInit();            // Deinitializer
Putty_Enum Putty_Reset();             // Reset all values
Putty_Enum Putty_Callback();          // Add it to a timer (or to the main loop)
Putty_Enum Putty_UARTCallback();      // Add it at HAL_UART_RxCpltCallback() | Called when data reception is finished

void Putty_SetRunTest(bool value);
bool Putty_GetRunTest();

#endif /* PUTTY_H_ */
