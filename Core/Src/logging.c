#include "logging.h"
#include "REM_BaseTypes.h"
#include "REM_BasestationLog.h"

#include "CircularBuffer.h"

#include <stdarg.h>
#include <string.h>
#include <stdio.h>

#include "usbd_cdc.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
extern USBD_HandleTypeDef hUsbDeviceFS;

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
    // Wait for USB to be ready for communication
    while(!(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED || hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED));
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
 * @brief Sends a log message over USB using the PACKET_TYPE_BASESTATION_LOG header.
 * The log must always end with \n, which this function enforces.
 * 
 * @param message The message to send over the USB
 */
void LOG(char *message){
    if(!LOG_canAddLog()) return;

    // Get message length
    uint32_t message_length = strlen(message);  
    // Clip the message length to 127 - PACKET_SIZE_REM_BASESTATION_LOG, as to not overflow the MessageContainer buffer
    if(127 - PACKET_SIZE_REM_BASESTATION_LOG < message_length) message_length = 127 - PACKET_SIZE_REM_BASESTATION_LOG;
    // Ensure newline at the end of the message (Can be removed if all software everywhere properly used the BasestationLog_messageLength field)
    message[message_length-1] = '\n';

    // Get the current write position, and increment ASAP, to (hopefully) prevent race conditions
    uint32_t index_write = buffer_indexer->indexWrite;
    CircularBuffer_write(buffer_indexer, NULL, 1);
    
    // Get the message container and its payload
    MessageContainer* message_container = &message_buffer[index_write];
    uint8_t* payload = message_container->payload;

    REM_BasestationLog_set_header((REM_BasestationLogPayload*) payload, PACKET_TYPE_REM_BASESTATION_LOG);  // 8 bits
    REM_BasestationLog_set_remVersion((REM_BasestationLogPayload*) payload, LOCAL_REM_VERSION);  // 4 bits
    REM_BasestationLog_set_messageLength((REM_BasestationLogPayload*) payload, message_length); // 8 bits
 
    // Copy the message into the message container, next to the BasestationLog header
    memcpy(payload + PACKET_SIZE_REM_BASESTATION_LOG, message, message_length);
    message_container->length = PACKET_SIZE_REM_BASESTATION_LOG + message_length;
    
}

static uint8_t buffer[100];

void LOG_send(){
    // Check if the USB is ready for transmitting
    if(!(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED || hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED)) return;
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    if (hcdc->TxState != 0) return;
    // Check if there is something in the buffer
    if(CircularBuffer_spaceFilled(buffer_indexer) == 0) return;
    // Write the message over USB
    MessageContainer* message_container = &message_buffer[buffer_indexer->indexRead];
    CDC_Transmit_FS(message_container->payload, message_container->length);

    // REM_BasestationLog_set_header((REM_BasestationLogPayload*) buffer, PACKET_TYPE_REM_BASESTATION_LOG);  // 8 bits
    // REM_BasestationLog_set_remVersion((REM_BasestationLogPayload*) buffer, LOCAL_REM_VERSION);  // 4 bits
    // REM_BasestationLog_set_messageLength((REM_BasestationLogPayload*) buffer, 5); // 8 bits
    // sprintf(buffer+3, "01234\n");
    // CDC_Transmit_FS(buffer, 8);

    // Move up the circular buffer
    CircularBuffer_read(buffer_indexer, NULL, 1);
}

void LOG_sendBlocking(uint8_t* data, uint8_t length){
    USBD_CDC_HandleTypeDef* hcdc = (USBD_CDC_HandleTypeDef*)hUsbDeviceFS.pClassData;
    while(!(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED || hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED));
	// if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED || hUsbDeviceFS.dev_state == USBD_STATE_SUSPENDED) {
    while(hcdc->TxState != 0);
		// TODO unneccesary memcpy? How about CDC_Transmit_FS(data, length);? 
		// 	 if CDC_Transmit_FS is blocking, no fear to have 'data' go out of scope.
		// memcpy(TxBuffer, data, length);
    CDC_Transmit_FS(data, length);
	// }
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