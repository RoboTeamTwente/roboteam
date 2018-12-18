#include <stdbool.h> // Bools
#include <string.h>  // Strings
#include <stdio.h>   // General

// // Files for extra commands
// #include "userIO/userIO.h"
// #include "Geneva/geneva.h"
// #include "DO/DO.h"
// #include "Ballsensor/ballsensor.h"
// #include "wheels/wheels.h"
// #include "kickchip/kickchip.h"
// #include "MTi/MTiControl.h"
// #include "wireless/wireless.h"
// #include "dribbler/dribbler.h"

#include "usart.h"
#include "PuTTY.h"
#include <stdint.h>
#include "main.h"
#include "gpio.h"

///////////////////////////////////////////////////// DEFINITIONS
// Commands to be remember
#define huartx huart3           // What huart communication is used
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

    HAL_UART_Receive_IT(&huartx, Putty_Vars.rec_buf, 1); // Data reception under serial interrupt mode

    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_GetErrorCode()
{
    return Putty_Vars.errorCode;
}

Putty_Enum Putty_printf(char *input)
{
    sprintf(Putty_Vars.smallStrBuffer, input); // Copies and turns into string
    Putty_TextOut(Putty_Vars.smallStrBuffer);  // Outputs to console
    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_DeInit()
{
    // TODO properly
    return Putty_Vars.errorCode = NoErrors;
}

Putty_Enum Putty_Reset()
{
    Putty_Init(Putty_Vars);
    return Putty_Vars.errorCode = NoErrors;
}

// Currently in main, but call it in a timer please.
// Note. Used to be called PuttyInterface_Update
Putty_Enum Putty_Callback()
{
    // If a command has been received
    if (Putty_Vars.huart_Rx_len)
    {
        Putty_HandlePcInput((char *)&Putty_Vars.small_buf, Putty_Vars.huart_Rx_len); // Deal with the PC Input
        Putty_Vars.huart_Rx_len = 0;                                                  // Reset the size of the input
        HAL_UART_Receive_IT(&huartx, Putty_Vars.rec_buf, 1);                          // Keep on checking for new data
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
    while (huartx.gState != HAL_UART_STATE_READY);            // Wait until ready
    memcpy(Putty_Vars.TxBuf, data, length);                  // Copy all data to TxBuf (Transmission Buffer)
    HAL_UART_Transmit_IT(&huartx, Putty_Vars.TxBuf, length); // Transmit the data to the compter
}

// Performs an action on the robot depending on the input.
static void Putty_HandleCommand(char *input)
{
    //// TODO: It is potentially easier to just type a number and have them in order of testing
    int inputNumber = *input - '0'; // Converts char input into int number.
    switch (inputNumber)
    {
    // Auto-testing
    case 0:
        HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, 1);
        break;
    // Geneva
    case 1:
    	// geneva_SetPosition(2 + strtol(input + 2, NULL, 10));
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

    // if (strcmp(input, "mt start") == 0)
    // {
    //     uprintf("Starting device MTi\n\r");
    //     if (MT_succes == MT_Init())
    //     {
    //         uprintf("MTi started.\n\r");
    //     }
    //     else
    //     {
    //         uprintf("No communication with MTi!\n\r");
    //     }
    // }
    // else if (!strcmp(input, "mt stop"))
    // {
    //     uprintf("resetting the MTi.\n\r");
    //     MT_DeInit();
    // }
    // else if (strcmp(input, "mt config") == 0)
    // {
    //     MT_GoToConfig();
    // }
    // else if (!strcmp(input, "mt measure"))
    // {
    //     MT_GoToMeasure();
    // }
    // else if (!strcmp(input, "mt options"))
    // {
    //     MT_ReqOptions();
    // }
    // else if (!strcmp(input, "mt setoptions"))
    // {
    //     MT_SetOptions();
    // }
    // else if (!strcmp(input, "mt icc"))
    // {
    //     MT_UseIcc();
    // }
    // else if (!strcmp(input, "mt norotation"))
    // {
    //     MT_NoRotation(10);
    // }
    // else if (!memcmp(input, SET_FILTER_COMMAND, strlen(SET_FILTER_COMMAND)))
    // {
    //     MT_SetFilterProfile(strtol(input + 1 + strlen(SET_FILTER_COMMAND), NULL, 10));
    // }
    // else if (strcmp(input, "mt factoryreset") == 0)
    // {
    //     uprintf("Resetting the configuration.\n\r");
    //     MT_FactoryReset();
    // }
    // else if (strcmp(input, "mt reqfilter") == 0)
    // {
    //     uprintf("requesting current filter profile.\n\r");
    //     MT_ReqFilterProfile();
    // }
    // else if (memcmp(input, "mt setconfig", strlen("mt setconfig")) == 0)
    // {
    //     MT_BuildConfig(XDI_PacketCounter, 100, false);
    //     MT_BuildConfig(XDI_FreeAcceleration, 100, false);
    //     MT_BuildConfig(XDI_EulerAngles, 100, true);
    // }
    // else if (strcmp(input, "reqconfig") == 0)
    // {
    //     uprintf("requesting output configuration mode\n\r");
    //     MT_RequestConfig();
    // }
    // else if (!strcmp(input, "address"))
    // {
    //     uprintf("address = [%d]\n\r", localRobotID);
    // }
    // else if (!strcmp(input, "example2"))
    // {
    //     uprintf("stop!\n\r");
    // }
    // else if (!strcmp(input, "geneva get"))
    // {
    //     uprintf("position = [%u]\n\r", geneva_GetPosition());
    // }
    // else if (!strcmp(input, "geneva stop"))
    // {
    //     geneva_SetState(geneva_idle);
    // }
    // else if (!strcmp(input, "euler"))
    // {
    //     print_euler = !print_euler;
    // }
    // else if (!memcmp(input, "geneva set", strlen("geneva set")))
    // {
    //     geneva_SetPosition(2 + strtol(input + 1 + strlen("geneva set"), NULL, 10));
    // }
    // else if (!memcmp(input, "control", strlen("control")))
    // {
    //     geneva_SetPosition(2 + strtol(input + 1 + strlen("control"), NULL, 10));
    // }
    // else if (!memcmp(input, "kick", strlen("kick")))
    // {
    //     kick_Shoot(strtol(input + 1 + strlen("kick"), NULL, 10), KICK);
    // }
    // else if (!memcmp(input, "chip", strlen("chip")))
    // {
    //     kick_Shoot(strtol(input + 1 + strlen("chip"), NULL, 10), CHIP);
    // }
    // else if (!memcmp(input, "block", strlen("block")))
    // {
    //     kick_Stateprint();
    // }
    // else if (!memcmp(input, TEST_WHEELS_COMMAND, strlen(TEST_WHEELS_COMMAND)))
    // {
    //     wheels_testing_power = atoff(input + strlen(TEST_WHEELS_COMMAND));
    //     wheels_testing = (wheels_testing_power <= -10 || wheels_testing_power >= 10);
    //     if ((wheels_testing))
    //     {
    //         uprintf("wheels test on, pwm [%f]\n\r", wheels_testing_power);
    //     }
    // }
    // else if (!memcmp(input, "dribble", strlen("dribble")))
    // {
    //     uint8_t speed = strtol(input + strlen("dribble"), NULL, 10);
    //     dribbler_SetSpeed(speed);
    //     uprintf("speed is set to[%lu]\n\r", __HAL_TIM_GET_COMPARE(&htim11, TIM_CHANNEL_1));
    // }

    // else if (!strcmp(input, "keyboard control"))
    // {
    //     uprintf("going to keyboard control\r\npress escape to stop\n\r");
    //     keyboard_control = true;
    //     wheels_testing = true;
    // }
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
    for (unsigned int i = 0; i < MAX_COMMAND_LENGTH; i++)
        Putty_TextOut(" "); // Input spacing (no idea why)
    Putty_TextOut("\r");    // Inputs return again
}
