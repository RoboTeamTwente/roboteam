/*
Author: Antonio
Date: 26/10/2018

Description: PuTTy file for communication with the PC via USART

Instructions:

Extra functions:

Error Codes Explanation:
0. No errors
1. 
2. 

GPIO Pins:

Notes:

Removed all USB capabilities. Seems more of a pain to keep than anything else.
 */
#ifndef PUTTYINTERFACE_H_
#define PUTTYINTERFACE_H_

#include <stdbool.h> // Bools
#include <string.h>  // Strings
#include <stdio.h>   // General

#include "usart.h"
///////////////////////////////////////////////////// DEFINITIONS
// Commands to be remember
#define huartx huart3 // What huart communication is used
#define COMMANDS_TO_REMEMBER 16 // Used in PC commands 2D matrix
#define MAX_COMMAND_LENGTH 32

///////////////////////////////////////////////////// VARIABLE STRUCT
typedef struct {
    char smallStrBuffer[1024];
    uint8_t TxBuf[1024];

    uint huart_Rx_len; // TODO: What's this?

    int errorCode; // Error codes
} Putty_Vars;


///////////////////////////////////////////////////// FUNCTION PROTOTYPES
// PUBLIC
void Putty_Printf(char * input); // Like printf
void Putty_Init(PuttyInterfaceTypeDef* pitd);
void Putty_DeInit();
void Putty_Reset();
void Putty_UARTCallback(PuttyInterfaceTypeDef* pitd);
void Putty_Callback(PuttyInterfaceTypeDef *pitd);

// PRIVATE
void Putty_TextOut(char *str); // Displays the code on the console
void Putty_HexOut(uint8_t data[], uint8_t length); // TODO: No idea really
void Putty_HandleCommand(char* input); // Executes action depending on input command
static void Putty_HandlePcInput(char *input, size_t n_chars, HandleLine func); // Called with timer | Change this
static uint8_t Putty_Wrap(uint8_t val, int8_t dif, uint8_t modulus); // Keeps values within the real range
static void Putt_ClearLine(); //Clears the current line to that new text can be placed.

///////////////////////////////////////////////////// FUNCTION IMPLEMENTATIONS (move to .c)
void Putty_printf (char* input){
    sprintf(smallStrBuffer, input); // Copies and turns into string
    TextOut(smallStrBuffer); // Outputs to console
}
void Putty_Init(PuttyInterfaceTypeDef *pitd)
{
	pitd->huart_Rx_len = 0;
	char *startmessage = "----------PuttyInterface_Init-----------\n\r";
	Putty_printf(startmessage);
    
	HAL_UART_Receive_IT(&huartx, pitd->rec_buf, 1);
}

// Callback after UART communication.
// Note. Add it at HAL_UART_RxCpltCallback()
void Putty_UARTCallback(PuttyInterfaceTypeDef puttyIn, UART_HandleTypeDef *huart)
{
    puttyIn.huart_Rx_len = 1;
    puttyIn.small_buf[0] = *(huart->pRxBuffPtr - 1);
}

// Currently in main, but call it in a timer please.
// Note. Used to be called PuttyInterface_Update
void Putty_Callback(PuttyInterfaceTypeDef *pitd)
{
    if (pitd->huart_Rx_len)
    {
        HandlePcInput((char *)&pitd->small_buf, pitd->huart_Rx_len, pitd->handle);
        pitd->huart_Rx_len = 0;
        HAL_UART_Receive_IT(&huartx, pitd->rec_buf, 1);
    }
}


void Putty_HexOut(uint8_t data[], uint8_t length) {
	while (huartx.gState != HAL_UART_STATE_READY); // 
	memcpy(TxBuf, data, length);
	HAL_UART_Transmit_IT(&huartx, TxBuf, length);
}

// Performs an action on the robot depending on the input.
void Putty_HandleCommand(char* input){
    int inputNumber = *input - '0'; // Converts char input into int number.
    switch (inputNumber){
        // Auto-testing
        case 0:
            break;
        // Geneva
        case 1:
            break;
        // Kick/Chip
        case 2:
            break;
        // Dribble
        case 3:
            break;
        // Wheels
        case 4:
            break;
        // Ball sensor
        case 5:
            break;
    }
}




//////////////// TO EDIT

// modulo keeping the value within the real range
// val is the start value,
// dif is the difference that will be added
// modulus is the value at which it wraps
static void ClearLine()
{
	TextOut("\r");
	for (uint i = 0; i < MAX_COMMAND_LENGTH; i++)
		TextOut(" ");
	TextOut("\r");
}

static uint8_t wrap(uint8_t val, int8_t dif, uint8_t modulus)
{
	dif %= modulus if (dif < 0)
		dif += modulus;
	dif += (val);
	if (dif >= modulus)
		dif -= modulus;
	return (uint8_t)dif;
}


void TextOut(char *str)
{
	uint8_t length = strlen(str);
	HexOut((uint8_t *)str, length);
}

void PuttyInterface_Init(PuttyInterfaceTypeDef *pitd)
{
	pitd->huart_Rx_len = 0;
	char *startmessage = "----------PuttyInterface_Init-----------\n\r";
	uprintf(startmessage);
#ifdef PUTTY_USART
	HAL_UART_Receive_IT(&huartx, pitd->rec_buf, 1);
#endif
}

static void HandlePcInput(char *input, size_t n_chars, HandleLine func)
{
	static char PC_Input[COMMANDS_TO_REMEMBER][MAX_COMMAND_LENGTH]; // Matrix which holds the entered commands
	static uint8_t PC_Input_counter = 0;							// counts the letters in the current forming command
	static int8_t commands_counter = 0;								// counts the entered commands
	static int8_t kb_arrow_counter = 0;								// counts the offset from the last entered command
	static uint8_t commands_overflow = 0;							// checks if there are COMMANDS_TO_REMEMBER commands stored
	if (input[0] == 0x0d)
	{						  //newline, is end of command
		kb_arrow_counter = 0; // reset the arrow input counter
		PC_Input[commands_counter][PC_Input_counter++] = '\0';
		TextOut("\r");
		TextOut(PC_Input[commands_counter]);
		TextOut("\n\r");
		PC_Input_counter = 0;
		func(PC_Input[commands_counter++]);																		// Callback func
		commands_overflow = !(commands_counter = commands_counter % COMMANDS_TO_REMEMBER) || commands_overflow; // if there are more than the maximum amount of stored values, this needs to be known
		PC_Input[commands_counter][0] = '\0';
	}
	else if (input[0] == 0x08)
	{ //backspace
		if (PC_Input_counter != 0)
			PC_Input_counter--;
	}
	else if (input[0] == 0x1b)
	{ //escape, also used for special keys in escape sequences
		if (n_chars > 1)
		{ // an escape sequence
			switch (input[1])
			{
			case 0x5b: // an arrow key
				switch (input[2])
				{
				case 'A': //arrow ^
					kb_arrow_counter--;
					break;
				case 'B': //arrow \/;
					kb_arrow_counter++;
					break;
				case 'C': //arrow ->
					break;
				case 'D': //arrow <-
					break;
				}
				uint8_t cur_pos = commands_overflow ? wrap(commands_counter, kb_arrow_counter, COMMANDS_TO_REMEMBER) : wrap(commands_counter, kb_arrow_counter, commands_counter + 1);
				PC_Input_counter = strlen(PC_Input[cur_pos]);
				strcpy(PC_Input[commands_counter], PC_Input[cur_pos]);
				ClearLine();
				TextOut(PC_Input[commands_counter]);
				break;
			}
		}
	}
	else
	{ // If it is not a special character, the value is put in the current string
		if (PC_Input_counter >= MAX_COMMAND_LENGTH)
		{
			ClearLine();
			uprintf("ERROR: command too long\n\r");
			memset(PC_Input[commands_counter], 0, MAX_COMMAND_LENGTH);
			PC_Input_counter = 0;
		}
		else
		{
			uprintf("%c", input[0]);
			PC_Input[commands_counter][PC_Input_counter++] = (char)input[0];
		}
	}
}
#endif