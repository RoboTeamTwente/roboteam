/*
Refactoring: Antonio

Date: 26/10/2018

Description: PuTTy file for communication with the PC via USART

Instructions:
1) Initialize it.
2) Place the Putty_Callback() function in loop or timer.
3) Place Putty_UARTCallbac() in HAL_UART_RxCpltCallback() 
4) Use PuTTy with the right communication port and a baud rate of 115200.

Extra functions:

GPIO Pins: None

Notes:
- There was code for USB, but it was never used. If we want to use it, we can just go to last year's repo
- Removed all USB capabilities. Seems more of a pain to keep than anything else.
 */

#ifndef PUTTYINTERFACE_H_
#define PUTTYINTERFACE_H_

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
    uint huart_Rx_len;         // Length of received data
    Putty_Enum errorCode;      // Error codes
} Putty_VarStruct;

Putty_VarStruct Putty_Vars; // Global struct with all variables

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC
Putty_Enum Putty_Init();              // Also returns a pointer to the variable struct
Putty_Enum Putty_GetErrorCode();      // Returns the error code
Putty_Enum Putty_printf(char *input); // Like printf, but for the putty console
Putty_Enum Putty_DeInit();            // Deinitializer
Putty_Enum Putty_Reset();             // Reset all values
Putty_Enum Putty_Callback();          // Add it to a timer (or to the main loop)
Putty_Enum Putty_UARTCallback();      // Add it at HAL_UART_RxCpltCallback() | Called when data reception is finished

#endif