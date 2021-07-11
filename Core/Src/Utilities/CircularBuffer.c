#include "CircularBuffer.h"
#include <string.h>

void CircularBuffer_init(CircularBuffer* circBuf){
    circBuf->indexWrite = 0;
    circBuf->indexRead = 0;
    for(uint32_t i = 0; i < CIRCULAR_BUFFER_SIZE; i++)
        // 46 is the ASCII code for dot(.) Done to make testing more clear
        circBuf->buffer[i] = 46;
}

uint32_t CircularBuffer_spaceFree(CircularBuffer* circBuf){
    if(circBuf->indexWrite < circBuf->indexRead)
        return circBuf->indexRead - circBuf->indexWrite - 1;
    else
        return CIRCULAR_BUFFER_SIZE - (circBuf->indexWrite - circBuf->indexRead) - 1;
}

uint32_t CircularBuffer_spaceFilled(CircularBuffer* circBuf){
    if(circBuf->indexWrite < circBuf->indexRead)
        return (circBuf->indexWrite + CIRCULAR_BUFFER_SIZE) - circBuf->indexRead;
    else
        return circBuf->indexWrite - circBuf->indexRead;
}

bool CircularBuffer_canWrite(CircularBuffer* circBuf, uint32_t length){
    return length <= CircularBuffer_spaceFree(circBuf);
}

bool CircularBuffer_canRead(CircularBuffer* circBuf, uint32_t length){
    return length <= CircularBuffer_spaceFilled(circBuf);
}

bool CircularBuffer_write(CircularBuffer* circBuf, uint8_t* data, uint32_t length){

    bool overflow = false;      // Boolean indicating an overflow occurred
    bool turnedAround = false;  // Boolean indicating if the writeIndex has been reset to 0. Used for overflow detection

    // Used for overflow detection
    bool writeAhead1 = circBuf->indexRead <= circBuf->indexWrite;

    // Data won't fit in the buffer. Only store the last CIRCULAR_BUFFER_SIZE bytes of the data
    if(CIRCULAR_BUFFER_SIZE < length){
        // Move up the pointer of the data
        data += length - CIRCULAR_BUFFER_SIZE;
        // Set the number of bytes to write to CIRCULAR_BUFFER_SIZE
        length = CIRCULAR_BUFFER_SIZE;
        overflow = true;
    }


    // Check if writing the message would overflow the buffer
    if(circBuf->indexWrite + length <= CIRCULAR_BUFFER_SIZE){
        memcpy(circBuf->buffer + circBuf->indexWrite, data, length);
    }else{
        uint32_t bytesAtEnd = CIRCULAR_BUFFER_SIZE - circBuf->indexWrite;     
        uint32_t bytesAtBegin = length - bytesAtEnd;
        // Copy first part of the data to the end of the buffer
        memcpy(circBuf->buffer + circBuf->indexWrite, data, bytesAtEnd);
        // Copy second part of the data to the beginning of the buffer
        memcpy(circBuf->buffer, data + bytesAtEnd, bytesAtBegin);
        turnedAround = true;
    }

    /* Check if an overflow has happened
     * writeAhead1 | writeAhead2 | turnedAround | interpretation | Example buffer drawing
     *1    false   |      false  |       false  | No overflow      ..W...R..... => ....W.R.....
     *2    false   |      false  |       true   | Overflow         ..W...R..... => .W....R..... => enough bytes to pass Read and turn around. Data possibly too large for buffer
     *3    false   |      true   |       false  | Overflow         ..W...R..... => ......R.W... => enough bytes to pass Read
     *4    false   |      true   |       true   | Overflow         CIRCULAR_BUFFER_SIZE < length. Data too large for buffer
     *5    true    |      false  |       false  | Impossible       Write cannot move backwards without turning around
     *6    true    |      false  |       true   | No overflow      ......R.W... => ..W...R.....
     *7    true    |      true   |       false  | No overflow      ......R.W... => ......R...W.
     *8    true    |      true   |       true   | Overflow         ......R...W. => ......R.W... => enough bytes written to turn around and pass Read
    */

    // Update indexWrite
    circBuf->indexWrite = (circBuf->indexWrite + length) % CIRCULAR_BUFFER_SIZE;
    // Used for overflow detection
    bool writeAhead2 = circBuf->indexRead <= circBuf->indexWrite;

    if(!writeAhead1 && !writeAhead2 &&  turnedAround) overflow = true; // Case 2
    if(!writeAhead1 &&  writeAhead2 && !turnedAround) overflow = true; // Case 3
    if(!writeAhead1 &&  writeAhead2 && !turnedAround) overflow = true; // Case 4
    if( writeAhead1 &&  writeAhead2 &&  turnedAround) overflow = true; // Case 8

    return overflow;
}

bool CircularBuffer_read(CircularBuffer* circBuf, uint8_t* buffer, uint32_t length){

    bool overrun = false;      // Boolean indicating an overrun occurred
    bool turnedAround = false;  // Boolean indicating if the readIndex has been reset to 0. Used for overrun detection

    // Used for overflow detection
    bool readAhead1 = circBuf->indexWrite < circBuf->indexRead;

    // More data requested than available in the buffer. Only read the last CIRCULAR_BUFFER_SIZE bytes of the data
    if(CIRCULAR_BUFFER_SIZE < length){
        // Set the number of bytes to read to CIRCULAR_BUFFER_SIZE
        length = CIRCULAR_BUFFER_SIZE;
        overrun = true;
    }

    // Check if reading would overrun the buffer
    if(circBuf->indexRead + length <= CIRCULAR_BUFFER_SIZE){
        memcpy(buffer, circBuf->buffer + circBuf->indexRead, length);
    }else{
        uint32_t bytesAtEnd = CIRCULAR_BUFFER_SIZE - circBuf->indexRead;
        uint32_t bytesAtBegin = length - bytesAtEnd;
        // Copy first part of the data to the end of the buffer
        memcpy(buffer, circBuf->buffer + circBuf->indexRead, bytesAtEnd);
        // Copy second part of the data to the beginning of the buffer
        memcpy(buffer + bytesAtEnd, circBuf->buffer, bytesAtBegin);
        turnedAround = true;
    }

    /* Check if an overrun has happened
     * readAhead1 | readAhead2 | turnedAround | interpretation | Example buffer drawing
     *1   false   |     false  |       false  | No overrun      ..R...W..... => ....R.W.....
     *2   false   |     false  |       true   | overrun         ..R...W..... => .R....W..... => enough bytes to pass Write and turn around. Buffer possibly too small for bytes requested
     *3   false   |     true   |       false  | overrun         ..R...W..... => ......W.R... => enough bytes to pass Read
     *4   false   |     true   |       true   | overrun         CIRCULAR_BUFFER_SIZE < length. Buffer possibly too small for bytes requested
     *5   true    |     false  |       false  | Impossible      Read cannot move backwards without turning around
     *6   true    |     false  |       true   | No overrun      ......W.R... => ..R...W.....
     *7   true    |     true   |       false  | No overrun      ......W.R... => ......W...R.
     *8   true    |     true   |       true   | overrun         ......W...R. => ......W.R... => enough bytes requested to turn around and pass Write.
    */

    // Update indexWrite
    circBuf->indexRead = (circBuf->indexRead + length) % CIRCULAR_BUFFER_SIZE;
    // Used for overrun detection
    bool readAhead2 = circBuf->indexWrite < circBuf->indexRead;

    if(!readAhead1 && !readAhead2 &&  turnedAround) overrun = true; // Case 2
    if(!readAhead1 &&  readAhead2 && !turnedAround) overrun = true; // Case 3
    if(!readAhead1 &&  readAhead2 && !turnedAround) overrun = true; // Case 4
    if( readAhead1 &&  readAhead2 &&  turnedAround) overrun = true; // Case 8

    return overrun;
}

/* ================================= Test functions below ================================= */
/*
void printCircBuf(CircularBuffer* circBuf){
    uint32_t space = CircularBuffer_spaceFree(circBuf);
    printf("\nCircular buffer. Read=%u write=%u space=%u\n", circBuf->indexRead, circBuf->indexWrite, space);
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE; i++){
        if(i == circBuf->indexWrite) printf("W");
        else printf(" ");
    }
    printf("\n");
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE; i++){
        printf("%c", circBuf->buffer[i]);
    }
    printf("\n");
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE; i++){
        if(i == circBuf->indexRead) printf("R");
        else printf(" ");;
    }
    printf("\n");
}

void printAndResetBuf(uint8_t* buf, uint32_t length){
    for(int i = 0; i < length; i++){
        printf("%c", buf[i]);
        buf[i] = '.';
    }
    printf("\n");
}

void testWrite(){
    CircularBuffer cb;

    uint8_t buffer[CIRCULAR_BUFFER_SIZE*2];
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE*2; i++) buffer[i] = i+65;
    bool overflow = false;

    // These tests have been written for a circular buffer with size 20;
    static_assert(CIRCULAR_BUFFER_SIZE == 20, "CIRCULAR_BUFFER_SIZE != 20");

    // case 1
    CircularBuffer_init(&cb); cb.indexWrite = 5; cb.indexRead = 15;
    printf("============================= Case 1\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(CircularBuffer_canWrite(&cb, 5));
    overflow = CircularBuffer_write(&cb, buffer, 5);
    printCircBuf(&cb);
    assert(!overflow);
    assert(cb.indexWrite == 10);
    assert(CircularBuffer_spaceFilled(&cb) == 15);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 2
    CircularBuffer_init(&cb); cb.indexWrite = 5; cb.indexRead = 15;
    printf("============================= Case 2\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(!CircularBuffer_canWrite(&cb, CIRCULAR_BUFFER_SIZE-3));
    overflow = CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE - 3);
    printCircBuf(&cb);
    assert(overflow);
    assert(cb.indexWrite == 2);
    assert(CircularBuffer_spaceFilled(&cb) == 7);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 3
    CircularBuffer_init(&cb); cb.indexWrite = 5; cb.indexRead = 15;
    printf("============================= Case 3\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(!CircularBuffer_canWrite(&cb, 12));
    overflow = CircularBuffer_write(&cb, buffer, 12);
    printCircBuf(&cb);
    assert(overflow);
    assert(cb.indexWrite == 17);
    assert(CircularBuffer_spaceFilled(&cb) == 2);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 4
    CircularBuffer_init(&cb); cb.indexWrite = 5; cb.indexRead = 10;
    printf("============================= Case 4\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 4);
    assert(!CircularBuffer_canWrite(&cb, CIRCULAR_BUFFER_SIZE + 10));
    overflow = CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE + 10);
    printCircBuf(&cb);
    assert(overflow);
    assert(cb.indexWrite == 5);
    assert(CircularBuffer_spaceFilled(&cb) == 15);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 5 => impossible

    // case 6
    CircularBuffer_init(&cb); cb.indexWrite = 15; cb.indexRead = 10;
    printf("============================= Case 6\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 14);
    assert(CircularBuffer_canWrite(&cb, 10));
    overflow = CircularBuffer_write(&cb, buffer, 10);
    printCircBuf(&cb);
    assert(!overflow);
    assert(cb.indexWrite == 5);
    assert(CircularBuffer_spaceFilled(&cb) == 15);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 7
    CircularBuffer_init(&cb); cb.indexWrite = 15; cb.indexRead = 10;
    printf("============================= Case 7\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 14);
    assert(CircularBuffer_canWrite(&cb, 3));
    overflow = CircularBuffer_write(&cb, buffer, 3);
    printCircBuf(&cb);
    assert(!overflow);
    assert(cb.indexWrite == 18);
    assert(CircularBuffer_spaceFilled(&cb) == 8);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 8
    CircularBuffer_init(&cb); cb.indexWrite = 15; cb.indexRead = 10;
    printf("============================= Case 8\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 14);
    assert(!CircularBuffer_canWrite(&cb, CIRCULAR_BUFFER_SIZE-2));
    overflow = CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE - 2);
    printCircBuf(&cb);
    assert(overflow);
    assert(cb.indexWrite == 13);
    assert(CircularBuffer_spaceFilled(&cb) == 3);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    printf("All write tests passed\n\n");
}

void testRead(){
    CircularBuffer cb;

    uint8_t buffer[CIRCULAR_BUFFER_SIZE*2];
    uint8_t readBuffer[CIRCULAR_BUFFER_SIZE*2];
    for(int i = 0; i < CIRCULAR_BUFFER_SIZE*2; i++) {
        buffer[i] = i + 65;
        readBuffer[i] = '.';
    }

    bool overrun = false;

    // These tests have been written for a circular buffer with size 20;
    static_assert(CIRCULAR_BUFFER_SIZE == 20, "CIRCULAR_BUFFER_SIZE !- 20");

    // case 1
    CircularBuffer_init(&cb); cb.indexRead = 5; cb.indexWrite = 15;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 1\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(CircularBuffer_canRead(&cb, 5));
    overrun = CircularBuffer_read(&cb, readBuffer, 5);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, 5);
    assert(!overrun);
    assert(cb.indexRead == 10);
    assert(CircularBuffer_spaceFilled(&cb) == 5);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 2
    CircularBuffer_init(&cb); cb.indexRead = 5; cb.indexWrite = 15;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 2\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(!CircularBuffer_canRead(&cb, CIRCULAR_BUFFER_SIZE-3));
    overrun = CircularBuffer_read(&cb, readBuffer, CIRCULAR_BUFFER_SIZE - 3);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, CIRCULAR_BUFFER_SIZE - 3);
    assert(overrun);
    assert(cb.indexRead == 2);
    assert(CircularBuffer_spaceFilled(&cb) == 13);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 3
    CircularBuffer_init(&cb); cb.indexRead = 5; cb.indexWrite = 15;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 3\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 9);
    assert(!CircularBuffer_canRead(&cb, 12));
    overrun = CircularBuffer_read(&cb, readBuffer, 12);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, 12);
    assert(overrun);
    assert(cb.indexRead == 17);
    assert(CircularBuffer_spaceFilled(&cb) == 18);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 4
    CircularBuffer_init(&cb); cb.indexRead = 5; cb.indexWrite = 10;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 4\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 14);
    assert(!CircularBuffer_canRead(&cb, CIRCULAR_BUFFER_SIZE + 10));
    overrun = CircularBuffer_read(&cb, readBuffer, CIRCULAR_BUFFER_SIZE + 10);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, CIRCULAR_BUFFER_SIZE + 10);
    assert(overrun);
    assert(cb.indexRead == 5);
    assert(CircularBuffer_spaceFilled(&cb) == 5);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 5 => impossible

    // case 6
    CircularBuffer_init(&cb); cb.indexRead = 15; cb.indexWrite = 10;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 6\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 4);
    assert(CircularBuffer_canRead(&cb, 10));
    overrun = CircularBuffer_read(&cb, readBuffer, 10);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, 10);
    assert(!overrun);
    assert(cb.indexRead == 5);
    assert(CircularBuffer_spaceFilled(&cb) == 5);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 7
    CircularBuffer_init(&cb); cb.indexRead = 15; cb.indexWrite = 10;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 7\n");
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 4);
    assert(CircularBuffer_canRead(&cb, 3));
    overrun = CircularBuffer_read(&cb, readBuffer, 3);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, 3);
    assert(!overrun);
    assert(cb.indexRead == 18);
    assert(CircularBuffer_spaceFilled(&cb) == 12);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    // case 8
    CircularBuffer_init(&cb); cb.indexRead = 15; cb.indexWrite = 10;
    CircularBuffer_write(&cb, buffer, CIRCULAR_BUFFER_SIZE);
    printf("============================= Case 8\n");
    printAndResetBuf(readBuffer, 5);
    printCircBuf(&cb);
    assert(CircularBuffer_spaceFree(&cb) == 4);
    assert(!CircularBuffer_canRead(&cb, CIRCULAR_BUFFER_SIZE-2));
    overrun = CircularBuffer_read(&cb, readBuffer, CIRCULAR_BUFFER_SIZE - 2);
    printCircBuf(&cb);
    printAndResetBuf(readBuffer, CIRCULAR_BUFFER_SIZE-2);
    assert(overrun);
    assert(cb.indexRead == 13);
    assert(CircularBuffer_spaceFilled(&cb) == 17);
    assert(CircularBuffer_spaceFree(&cb) == CIRCULAR_BUFFER_SIZE - 1 - CircularBuffer_spaceFilled(&cb));

    printf("All read tests passed\n\n");
}

int main(int argc, char *argv[]){
    testWrite();
    testRead();
}
*/