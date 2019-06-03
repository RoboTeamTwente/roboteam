/*
 * PuTTY.c
 *
 *  Created on: 9 apr. 2019
 *      Author: Cas Doornkamp
 */

#include "PuTTY.h"
#include "peripheral_util.h"	// uart location
#include "stm32f7xx_hal.h"
#include <stdbool.h> // Bools
#include <string.h>  // Strings
#include <stdio.h>   // General
#include <stdarg.h>  // for formatting in putty_printf
#include <stdint.h>

///////////////////////////////////////////////////// DEFINITIONS
// Commands to be remember
#define COMMANDS_TO_REMEMBER 16 // Used in PC commands 2D matrix
#define MAX_COMMAND_LENGTH 32

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
static void Putty_TextOut(char *str);                                // Displays the code on the console
static void Putty_HexOut(uint8_t data[], uint8_t length);            // Transmits data to PC
static void Putty_HandleCommand(char *input);                        // Executes action depending on input command
static void Putty_HandlePcInput(char *input, size_t n_chars);        // Called with timer | Change this
static uint8_t Putty_Wrap(uint8_t val, int8_t dif, uint8_t modulus); // Keeps values within the real range
static void Putty_ClearLine();                                       //Clears the current line to that new text can be placed.

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS
Putty_Enum Putty_Init()
{

    Putty_Vars.errorCode = 0;
    Putty_Vars.huart_Rx_len = 0;

    char *startmessage = "----------PuttyInterface_Init-----------\n\r"; // Initial message
    Putty_printf(startmessage);

    HAL_UART_Receive_IT(UART_PC, Putty_Vars.rec_buf, 1); // Data reception under serial interrupt mode

    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_DeInit()
{
    // TODO properly
    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_GetErrorCode()
{
    return Putty_Vars.errorCode;
}

Putty_Enum Putty_printf(char *format, ...)
{
	//format string
	va_list aptr;
	va_start(aptr, format); // give starting point of additional arguments
    vsprintf(Putty_Vars.smallStrBuffer, format, aptr); // Copies and turns into string
    va_end(aptr); // close list

    // output string
    Putty_TextOut(Putty_Vars.smallStrBuffer);
    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_Reset()
{
    Putty_Init(Putty_Vars);
    return Putty_Vars.errorCode = NoErrors;
}


// Note. Used to be called PuttyInterface_Update
Putty_Enum Putty_Callback()
{
    // If a command has been received
    if (Putty_Vars.huart_Rx_len)
    {
        Putty_HandlePcInput((char *)&Putty_Vars.small_buf, Putty_Vars.huart_Rx_len); // Deal with the PC Input
        Putty_Vars.huart_Rx_len = 0;                                                  // Reset the size of the input
        HAL_UART_Receive_IT(UART_PC, Putty_Vars.rec_buf, 1);                          // Keep on checking for new data
    }
    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_UARTCallback(UART_HandleTypeDef *huart)
{
    Putty_Vars.huart_Rx_len = 1;                        // Reset the length received
    Putty_Vars.small_buf[0] = *(huart->pRxBuffPtr - 1); // Copies received data into buffer
    return Putty_Vars.errorCode;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
static void Putty_TextOut(char *str)
{
    uint8_t length = strlen(str);         // Gets the length of the data string
    Putty_HexOut((uint8_t *)str, length); // Transmits the data
}

static void Putty_HexOut(uint8_t data[], uint8_t length)
{
    while (UART_PC->gState != HAL_UART_STATE_READY);            // Wait until ready
    memcpy(Putty_Vars.TxBuf, data, length);                  // Copy all data to TxBuf (Transmission Buffer)
    HAL_UART_Transmit_DMA(UART_PC, Putty_Vars.TxBuf, length); // Transmit the data to the compter
}

// Performs an action on the robot depending on the input.
static void Putty_HandleCommand(char *input)
{
	if(!strcmp(input, "geneva get")){
		Putty_printf("Geneva encoder = %i\n\r", geneva_GetEncoder());
	} else if(!memcmp(input, "geneva set" , strlen("geneva set"))){
		geneva_SetRef(strtol(input + 1 + strlen("geneva set"), NULL, 10));
	}else if(!memcmp(input, "kick", strlen("kick"))){
		shoot_Shoot(shoot_Kick);
	}else if(!memcmp(input, "chip", strlen("chip"))){
		shoot_Shoot(shoot_Chip);
	}else if(!memcmp(input, "shoot power", strlen("shoot power"))){
		shoot_SetPower(strtol(input + 1 + strlen("shoot power"), NULL, 10));
	}else if(!strcmp(input, "shoot state")){
		Putty_printf("Shoot state = %i\n\r", shoot_GetState());
	}else if(!memcmp(input, "dribble", strlen("dribble"))){
		dribbler_SetSpeed(strtol(input + 1 + strlen("dribble"), NULL, 10));
	}else if(!memcmp(input, "wheels", strlen("wheels"))){
		float wheel = strtol(input + 1 + strlen("wheels"), NULL, 10);
		float wheelref[4] = {wheel, wheel, wheel, wheel};
		wheels_SetRef(wheelref);
	}else if(!strcmp(input, "make robots")){
		Putty_printf("No U!");
	}else if (!memcmp(input, "run full test", strlen("run full test"))) {
		test_RunTest(full);
	}else if (!memcmp(input, "run square test", strlen("run square test"))) {
		test_RunTest(square);
	}
	return;
}

static void Putty_HandlePcInput(char *input, size_t n_chars)
{
    static char PC_Input[COMMANDS_TO_REMEMBER][MAX_COMMAND_LENGTH]; // Matrix which holds the entered commands
    static uint8_t PC_Input_counter = 0;                            // counts the letters in the current forming command
    static int8_t commands_counter = 0;                             // counts the entered commands
    static int8_t kb_arrow_counter = 0;                             // counts the offset from the last entered command
    static uint8_t commands_overflow = 0;                           // checks if there are COMMANDS_TO_REMEMBER commands stored
    if (input[0] == 0x0d)
    {                         //newline, is end of command
        kb_arrow_counter = 0; // reset the arrow input counter
        PC_Input[commands_counter][PC_Input_counter++] = '\0';
        Putty_TextOut("\r");
        Putty_TextOut(PC_Input[commands_counter]);
        Putty_TextOut("\n\r");
        PC_Input_counter = 0;
        Putty_HandleCommand(PC_Input[commands_counter++]);
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
                uint8_t cur_pos = commands_overflow ? Putty_Wrap(commands_counter, kb_arrow_counter, COMMANDS_TO_REMEMBER) : Putty_Wrap(commands_counter, kb_arrow_counter, commands_counter + 1);
                PC_Input_counter = strlen(PC_Input[cur_pos]);
                strcpy(PC_Input[commands_counter], PC_Input[cur_pos]);
                Putty_ClearLine();
                Putty_TextOut(PC_Input[commands_counter]);
                break;
            }
        }
    }
    else
    { // If it is not a special character, the value is put in the current string
        if (PC_Input_counter >= MAX_COMMAND_LENGTH)
        {
            Putty_ClearLine();
            Putty_printf("ERROR: command too long\n\r");
            memset(PC_Input[commands_counter], 0, MAX_COMMAND_LENGTH);
            PC_Input_counter = 0;
        }
        else
        {
            Putty_printf("%c" /*, input[0]*/);
            PC_Input[commands_counter][PC_Input_counter++] = (char)input[0];
        }
    }
}

static uint8_t Putty_Wrap(uint8_t val, int8_t dif, uint8_t modulus)
{
    dif %= modulus; // Modulus of dif in terms of modulus

    if (dif < 0)
    {
        dif += modulus; // If the value is -ve, increase by the modulus
    }

    dif += (val); // Increase by the value

    if (dif >= modulus)
    {
        dif -= modulus; // Check that it does not exceed the modulus
    }

    return (uint8_t)dif; // Return the difference
}

static void Putty_ClearLine()
{
    Putty_TextOut("\r"); // Inputs "return" (i.e. take the cursor to the beginning of the line)
    for (unsigned int i = 0; i < MAX_COMMAND_LENGTH; i++){
        Putty_TextOut(" "); // Input spacing (no idea why)
    }
    Putty_TextOut("\r");    // Inputs return again
}
