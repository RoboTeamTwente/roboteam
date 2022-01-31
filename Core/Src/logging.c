#include "logging.h"
#include "BaseTypes.h"
#include "RobotLog.h"
#include "peripheral_util.h"
#include "robot.h"

#include <stdarg.h>  // LOG_printf formatting

// Static to make it private to this file
// Buffer used by vsprintf
static char printf_buffer[1024]; 
static char log_buffer[1024];
static RobotLogPayload robotLogPayload;

typedef struct _Message {
    uint8_t payload[127];
} Message;

static Message message_buffer[100];

void LOG_printf(char *format, ...)
{
	// Place variables into string
	va_list aptr; 
	va_start(aptr, format); // Give starting point of additional arguments
    vsprsintf(printf_buffer, format, aptr); // Copies and turns into string
    va_end(aptr); // Close list
    // Print the message
    LOG(printf_buffer);
}


/**
 * TODO Ensure that messages are not "too" long, whatever that may be.
 * 
 * @brief Sends a log message over USB using the PACKET_TYPE_ROBOT_LOG header.
 * The log must always end with \n, which this function enforces.
 * 
 * @param message The message to send over the USB
 */
void LOG(char *message){
    
    // Get message length
    uint32_t message_length = strlen(message);  
    // Ensure newline at the end of the message (Can be removed if all software everywhere properly used the RobotLog_message_length field)
    message[message_length-1] = '\n';

    RobotLog_set_header(&robotLogPayload, PACKET_TYPE_ROBOT_LOG); // 8 bits
    RobotLog_set_remVersion(&robotLogPayload, LOCAL_REM_VERSION); // 4 bits
    RobotLog_set_id(&robotLogPayload, ROBOT_ID);                  // 4 bits
    RobotLog_set_message_length(&robotLogPayload, message_length);// 8 bits
                                                                  // = 3 bytes

    // Copy the RobotLog packet into the buffer
    memcpy(log_buffer, &robotLogPayload, PACKET_SIZE_ROBOT_LOG);
    // Copy the message into the buffer, next to the RobotLog packet
    memcpy(log_buffer + PACKET_SIZE_ROBOT_LOG, message, message_length);
    // Wait until the UART interface is free to use
    while(UART_PC->gState != HAL_UART_STATE_READY);
    // Disabling and enabling all interrupts seems to fix the code from hanging when this code is called from an IRQ
    __disable_irq();
    // Transmit the data to the computer
    HAL_UART_Transmit(UART_PC, log_buffer, PACKET_SIZE_ROBOT_LOG + message_length, 10);
    __enable_irq();
    // HAL_UART_Transmit_DMA(UART_PC, log_buffer, robotlog_length + message_length);
}