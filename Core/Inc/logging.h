#ifndef LOGGING_H_
#define LOGGING_H_

/**
 * @file logging.h
 * @author Emiel Steerneman
 * @date 2022-05-09
 */

/**
 * @brief logging.c Is simply a wr  apper around a buffer, that stores messages and sends then out when possible
 * 
 * This entire logging thing consists of a large circular buffer where the messages are stored, a circular
 * buffer implementation which is only used to track where the actual buffer currently is, and a function
 * to send something over USB if possible. Each message is currently limited to 127 bytes (including the
 * REM_BasestationLog header), since that's the current limit of the SX1280. Each message also tracks its 
 * length with a uint32_t.
 * 
 * TODO : Support messages larger than 127 bytes, or automatically split up messages into chunks of max 127
 * bytes. If split up, figure out a way to track which packages belong together. 
 * TODO : Support selecting output channels (SX1280, USB, SD-card?, ...)
 */

// https://stackoverflow.com/questions/2670816/how-can-i-use-the-compile-time-constant-line-in-a-string/2671100
// https://gcc.gnu.org/onlinedocs/gcc-4.8.5/cpp/Stringification.html
// These two macros allow us to add __LINE__, __FILE__, etc to strings at compile-time
#define STRINGIZE_DETAIL(x) #x
#define STRINGIZE(x) STRINGIZE_DETAIL(x)

// Size of the circular buffer. This translates to 100 x (127+4) bytes
#define LOG_MAX_MESSAGES 100

// https://stm32f4-discovery.net/2015/06/get-interrupt-execution-status-on-cortex-m-processors/
// ^ Might be useful someday to check if we're currently in an IRQ. Never print in an IRQ unless it's blocking

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Reserve space for the circular buffer, initialize the indexer, and wait for the USB to be initialized
 */
void LOG_init();

/**
 * @brief Place variables into a string, and move into buffer if possible
 * 
 * @param format The string, for example "Float at place %d is %.2f" 
 * @param ... The values, for example 12, 0.4327
 */
void LOG_printf(char *format, ...);

/**
 * @brief Places a message into the buffer if possible. Truncates to 127-PACKET_SIZE_REM_BASESTATION_LOG bytes
 * 
 * @param message The message to be placed in the buffer
 */
void LOG(char *message);

/**
 * @brief Send the next message over USB if USB is available 
 */
void LOG_send();

/**
 * @brief Force the 
 * 
 * @param data 
 * @param length 
 */
void LOG_sendBlocking(uint8_t* data, uint8_t length);

/**
 * @brief Check if a message can be added to the buffer. Buffer needs to be initialized and have free space
 */
bool LOG_canAddLog();

/**
 * @brief Send all messages in the buffer using a blocking while-loop
 */
void LOG_sendAll();

/**
 * @brief Check if there is currently a message in the buffer 
 */
bool LOG_hasMessage();

#endif /* LOGGING_H_ */

