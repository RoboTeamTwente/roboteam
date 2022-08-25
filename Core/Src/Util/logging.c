#include "robot.h"
#include "logging.h"
#include "REM_BaseTypes.h"
#include "REM_RobotLog.h"
#include "peripheral_util.h"

#include "CircularBuffer.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

// Buffer used by vsprintf. Static to make it private to this file
static char printf_buffer[1024];

typedef struct _MessageContainer {
    uint8_t length;
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
    // Wait for UART to be ready
    while(HAL_UART_GetState(UART_PC) != HAL_UART_STATE_READY);

    log_initialized = true;
}

void LOG_printf(char *format, ...){
    if(!LOG_canAddLog()) return;

	// Place variables into string
	va_list aptr; 
	va_start(aptr, format); // Give starting point of additional arguments
    vsnprintf(printf_buffer, 1024, format, aptr); // Copies and turns into string
    va_end(aptr); // Close list
    // Add the message to the buffer
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
    // Clip the message length to 127 - PACKET_SIZE_ROBOT_LOG, as to not overflow the MessageContainer buffer
    if(127 - PACKET_SIZE_REM_ROBOT_LOG < message_length) message_length = 127 - PACKET_SIZE_REM_ROBOT_LOG;
    // Ensure newline at the end of the message (Can be removed if all software everywhere properly used the RobotLog_message_length field)
    message[message_length-1] = '\n';

    // Get the current write position, and increment ASAP, to (hopefully) prevent race conditions
    uint32_t index_write = buffer_indexer->indexWrite;
    CircularBuffer_write(buffer_indexer, NULL, 1);
    
    // Get the message container and its payload
    MessageContainer* message_container = &message_buffer[index_write];
    uint8_t* payload = message_container->payload;

    REM_RobotLog_set_header((REM_RobotLogPayload*) payload, PACKET_TYPE_REM_ROBOT_LOG);  // 8 bits
    REM_RobotLog_set_remVersion((REM_RobotLogPayload*) payload, LOCAL_REM_VERSION);  // 4 bits
    REM_RobotLog_set_id((REM_RobotLogPayload*) payload, robot_get_ID());             // 4 bits
    REM_RobotLog_set_messageLength((REM_RobotLogPayload*) payload, message_length); // 8 bits
                                                                            // = 3 bytes
 
    // Copy the message into the message container, next to the RobotLog header
    memcpy(payload + PACKET_SIZE_REM_ROBOT_LOG, message, message_length);
    message_container->length = PACKET_SIZE_REM_ROBOT_LOG + message_length;
    
}

void LOG_send(){
    // Can't use HAL_UART_STATE_READY because it clashes with rem.c:HAL_UART_Receive_IT
    // So (for now), just check if it is busy transmitting
    if(HAL_UART_GetState(UART_PC) == HAL_UART_STATE_BUSY_TX) return;
    if(HAL_UART_GetState(UART_PC) == HAL_UART_STATE_BUSY_TX_RX) return;
    // Check if there is something in the buffer
    if(CircularBuffer_spaceFilled(buffer_indexer) == 0) return;

    MessageContainer* message_container = &message_buffer[buffer_indexer->indexRead];
    HAL_UART_Transmit_DMA(UART_PC, message_container->payload, message_container->length);
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