#ifndef __CIRCULARBUFFER_H
#define __CIRCULARBUFFER_H

#include <stdint.h>
#include <stdbool.h>

#define CIRCULAR_BUFFER_SIZE 20

/** For proper functioning of the circular buffer, ensure the following
 * Write can never be more than 0 to CIRCULAR_BUFFER_SIZE-1 ahead of Read
 * Read can only ever be CIRCULAR_BUFFER_SIZE-1 up to 0 behind Write
 * All code runs under the assumption that Write is either equal to Read or ahead of Read
 */

typedef struct _CircularBuffer {
    bool onlyTrackIndex;
    uint32_t indexWrite;
    uint32_t indexRead;
    uint32_t bufferSize;
    uint8_t buffer[];
} CircularBuffer;


CircularBuffer* CircularBuffer_init(bool onlyTrackIndex, uint32_t bufferSize);

/**
 * Returns the number of bytes that could still fit in the buffer.
 * Note that the maximum number of free space can never exceed CIRCULAR_BUFFER_SIZE-1 bytes
 * @param circBuf Pointer to the circular buffer object
 * @return the number of bytes of free space in the buffer in the range [0, CIRCULAR_BUFFER_SIZE-1]
 */
uint32_t CircularBuffer_spaceFree(CircularBuffer* circBuf);

/**
 * Returns the number of bytes that can be read from the buffer.
 * Note that the maximum number of bytes that can be read can never exceed CIRCULAR_BUFFER_SIZE-1
 * @param circBuf Pointer to the circular buffer object
 * @return the number of bytes that can be read from the buffer in the range [0, CIRCULAR_BUFFER_SIZE-1]
 */
uint32_t CircularBuffer_spaceFilled(CircularBuffer* circBuf);

/**
 * Check if a hypotethical message of 'length' bytes would fit in the buffer.
 * @param circBuf Pointer to the circular buffer object
 * @param length Length of the hypothetical message
 * @return True if it fits in the buffer, False if it does not fit in the buffer
 */
bool CircularBuffer_canWrite(CircularBuffer* circBuf, uint32_t length);

/**
 * Writes 'length' bytes to the buffer, even if this overflows.
 * @param circBuf Pointer to the circular buffer object
 * @param data The buffer where the data to be written is stored
 * @param length The number of bytes to be written to the buffer
 * @return True if an overflow occurred, False if no overflow occurred
 */
bool CircularBuffer_write(CircularBuffer* circBuf, uint8_t* data, uint32_t length);

/**
 * Reads 'length' bytes from the circular buffer into the provided buffer, even if this overruns.
 * @param circBuf Pointer to the circular buffer object
 * @param buffer The buffer where the data has to be written to
 * @param length The number of bytes to be written to the buffer
 * @return True if an overrun occurred, False if no overrun occurred
 */
bool CircularBuffer_read(CircularBuffer* circBuf, uint8_t* buffer, uint32_t length);

# endif /*__CIRCULARBUFFER_H*/