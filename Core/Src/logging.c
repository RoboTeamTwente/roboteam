#include "logging.h"
#include "BaseTypes.h"
#include "RobotLog.h"
#include "peripheral_util.h"
#include "robot.h"

#include "CircularBuffer.h"

#include <stdarg.h>  // LOG_printf formatting
#include <string.h>
#include <stdio.h>

// Buffer used by vsprintf. Static to make it private to this file
static char printf_buffer[1024]; 
static char log_buffer[1024];

typedef struct _MessageContainer {
    uint8_t payload[127];
} MessageContainer;

static MessageContainer message_buffer[LOG_MAX_MESSAGES];
CircularBuffer* buffer_indexer = NULL;
static bool log_initialized = false;

void LOG_init(){
    // Can't initialize twice
    if(log_initialized) return;
    // Create buffer indexer
    buffer_indexer = CircularBuffer_init(true, LOG_MAX_MESSAGES);

    while(HAL_UART_GetState(UART_PC) != HAL_UART_STATE_READY);

    sprintf(log_buffer, "LOG_init!\n");
    while(UART_PC->gState != HAL_UART_STATE_READY);
    HAL_UART_Transmit(UART_PC, log_buffer, strlen(log_buffer), 50);

    log_initialized = true;
}

void LOG_printf(char *format, ...){
    if(!LOG_canAddLog()) return;

	// Place variables into string
	va_list aptr; 
	va_start(aptr, format); // Give starting point of additional arguments
    vsprintf(printf_buffer, format, aptr); // Copies and turns into string
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
    if(!LOG_canAddLog()) return;

    // Get message length
    uint32_t message_length = strlen(message);  
    // Clip the message length to 127, as to not overflow the MessageContainer buffer
    if(127 < message_length) message_length = 127;
    // Ensure newline at the end of the message (Can be removed if all software everywhere properly used the RobotLog_message_length field)
    message[message_length-1] = '\n';

    MessageContainer* message_container = &message_buffer[buffer_indexer->indexWrite];
    CircularBuffer_write(buffer_indexer, NULL, 1);
    uint8_t* payload = message_container->payload;

    RobotLog_set_header((RobotLogPayload*) payload, PACKET_TYPE_ROBOT_LOG);  // 8 bits
    RobotLog_set_remVersion((RobotLogPayload*) payload, LOCAL_REM_VERSION);  // 4 bits
    RobotLog_set_id((RobotLogPayload*) payload, ROBOT_ID);                   // 4 bits
    RobotLog_set_message_length((RobotLogPayload*) payload, message_length); // 8 bits
                                                                                       // = 3 bytes
 
    // Copy the message into the message container, next to the RobotLog header
    memcpy(payload + PACKET_SIZE_ROBOT_LOG, message, message_length);
    
    // // Wait until the UART interface is free to use
    // while(UART_PC->gState != HAL_UART_STATE_READY);
    // // Disabling and enabling all interrupts seems to fix the code from hanging when this code is called from an IRQ
    // __disable_irq();
    // // Transmit the data to the computer
    // HAL_UART_Transmit(UART_PC, log_buffer, PACKET_SIZE_ROBOT_LOG + message_length, 10);
    // __enable_irq();
    // // HAL_UART_Transmit_DMA(UART_PC, log_buffer, robotlog_length + message_length);
}

void LOG_send(){
    // Can't use HAL_UART_STATE_READY because it clashes with rem.c:HAL_UART_Receive_IT
    // So (for now), just check if it is busy transmitting
    if(HAL_UART_GetState(UART_PC) == HAL_UART_STATE_BUSY_TX) return;
    if(HAL_UART_GetState(UART_PC) == HAL_UART_STATE_BUSY_TX_RX) return;
    // Check if there is something in the buffer
    if(CircularBuffer_spaceFilled(buffer_indexer) == 0) return;

    MessageContainer* message_container = &message_buffer[buffer_indexer->indexRead];
    uint32_t message_length = RobotLog_get_message_length((RobotLogPayload*) message_container);
    HAL_UART_Transmit_DMA(UART_PC, message_container, PACKET_SIZE_ROBOT_LOG + message_length);
    CircularBuffer_read(buffer_indexer, NULL, 1);

}

void LOG_sendAll(){
    while(LOG_hasMessage()) LOG_send();
}

bool LOG_hasMessage(){
    return 0 < CircularBuffer_spaceFilled(buffer_indexer);
}

bool LOG_canAddLog(){
    if(!log_initialized) return false;
    return CircularBuffer_canWrite(buffer_indexer, 1);
}