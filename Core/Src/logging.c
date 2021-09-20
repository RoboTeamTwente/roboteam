#include "logging.h"
#include "BaseTypes.h"
#include "peripheral_util.h"

#include <stdarg.h>  // LOG_printf formatting in putty_printf

// Static to make it private to this file
// Buffer used by vsprintf
static char printf_buffer[1024]; 

void LOG_printf(char *format, ...)
{
	// Place variables into string
	va_list aptr; 
	va_start(aptr, format); // Give starting point of additional arguments
    vsprintf(printf_buffer, format, aptr); // Copies and turns into string
    va_end(aptr); // Close list
    // Print the message
    LOG(printf_buffer);
}


/**
 * @brief Sends a log message over USB using the PACKET_TYPE_ROBOT_LOG header.
 * The log must always end with \n, which this function enforces.
 * 
 * @param message The message to send over the USB
 */
void LOG(char *message){
    // Add +1 to the length of the string to account for the extra header bytes
    int length = strlen(message) + 1;
    // Free up space for header + message
    uint8_t* buffer = malloc(length);
    // Set the header
    buffer[0] = PACKET_TYPE_ROBOT_LOG;
    // Enforce newline
    buffer[length] = '\n';
    // Copy the message into the buffer, next to the header
    memcpy(buffer+1, message, length-1);
    // Wait until the UART interface is free to use
    while(UART_PC->gState != HAL_UART_STATE_READY);
    // Transmit the data to the computer
    HAL_UART_Transmit_DMA(UART_PC, buffer, length);
    // Free up the memory
    free(buffer);
}