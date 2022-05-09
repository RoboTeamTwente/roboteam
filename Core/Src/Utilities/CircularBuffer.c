#include "CircularBuffer.h"
#include <string.h>
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

CircularBuffer* CircularBuffer_init(bool onlyTrackIndex, uint32_t bufferSize){
    /** https://stackoverflow.com/questions/2060974/how-to-include-a-dynamic-array-inside-a-struct-in-c
     * Is this ugly? Yes. Do I care? No. This only works if buffer is of type uint_8 / char (anything
     * of size 1 byte) and "uint8_t buffer[];" is the last thing declared in the struct
     **/
    CircularBuffer* circBuf;
    if(onlyTrackIndex) circBuf = malloc(sizeof(CircularBuffer));
    else               circBuf = malloc(sizeof(CircularBuffer) + bufferSize);
    
    circBuf->onlyTrackIndex = onlyTrackIndex;
    circBuf->indexWrite = 0;
    circBuf->indexRead = 0;
    circBuf->bufferSize = bufferSize;
    
    if(!onlyTrackIndex){
        for(uint32_t i = 0; i < circBuf->bufferSize; i++)
            // 46 is the ASCII code for dot(.) Done to make testing more clear
            circBuf->buffer[i] = 46;
    }

    return circBuf;
}

uint32_t CircularBuffer_spaceFree(CircularBuffer* circBuf){
    if(circBuf->indexWrite < circBuf->indexRead)
        return circBuf->indexRead - circBuf->indexWrite - 1;
    else
        return circBuf->bufferSize - (circBuf->indexWrite - circBuf->indexRead) - 1;
}

uint32_t CircularBuffer_spaceFilled(CircularBuffer* circBuf){
    if(circBuf->indexWrite < circBuf->indexRead)
        return (circBuf->indexWrite + circBuf->bufferSize) - circBuf->indexRead;
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

    bool overflow = false;       // Boolean indicating an overflow occurred
    bool wrappedAround = false;  // Boolean indicating if the writeIndex has been reset to 0. Used for overflow detection

    // Used for overflow detection
    bool writeAhead1 = circBuf->indexRead <= circBuf->indexWrite;

    // Data won't fit in the buffer. Only store the last circBuf->bufferSize bytes of the data
    if(circBuf->bufferSize < length){
        // Move up the pointer of the data
        data += length - circBuf->bufferSize;
        // Set the number of bytes to write to circBuf->bufferSize
        length = circBuf->bufferSize;
        overflow = true;
    }

    // Check if writing the message would overflow the buffer. If so, wrapping is needed
    wrappedAround = circBuf->bufferSize < circBuf->indexWrite + length;

    // Only write if the buffer is actually initialized
    if(!circBuf->onlyTrackIndex){
        if(wrappedAround){
            uint32_t bytesAtEnd = circBuf->bufferSize - circBuf->indexWrite;     
            uint32_t bytesAtBegin = length - bytesAtEnd;
            // Copy first part of the data to the end of the buffer
            memcpy(circBuf->buffer + circBuf->indexWrite, data, bytesAtEnd);
            // Copy second part of the data to the beginning of the buffer
            memcpy(circBuf->buffer, data + bytesAtEnd, bytesAtBegin);
            wrappedAround = true;
        }else{
            memcpy(circBuf->buffer + circBuf->indexWrite, data, length);
        }
    }

    

    /* Check if an overflow has happened
     * writeAhead1 | writeAhead2 | wrappedAround | interpretation | Example buffer drawing
     *1    false   |      false  |       false  | No overflow      ..W...R..... => ....W.R.....
     *2    false   |      false  |       true   | Overflow         ..W...R..... => .W....R..... => enough bytes to pass Read and wrap around. Data possibly too large for buffer
     *3    false   |      true   |       false  | Overflow         ..W...R..... => ......R.W... => enough bytes to pass Read
     *4    false   |      true   |       true   | Overflow         circBuf->bufferSize < length. Data too large for buffer
     *5    true    |      false  |       false  | Impossible       Write cannot move backwards without wrapping around
     *6    true    |      false  |       true   | No overflow      ......R.W... => ..W...R.....
     *7    true    |      true   |       false  | No overflow      ......R.W... => ......R...W.
     *8    true    |      true   |       true   | Overflow         ......R...W. => ......R.W... => enough bytes written to wrap around and pass Read
    */

    // Update indexWrite
    circBuf->indexWrite = (circBuf->indexWrite + length) % circBuf->bufferSize;
    // Used for overflow detection
    bool writeAhead2 = circBuf->indexRead <= circBuf->indexWrite;

    if(!writeAhead1 && !writeAhead2 &&  wrappedAround) overflow = true; // Case 2
    if(!writeAhead1 &&  writeAhead2 && !wrappedAround) overflow = true; // Case 3
    if(!writeAhead1 &&  writeAhead2 && !wrappedAround) overflow = true; // Case 4
    if( writeAhead1 &&  writeAhead2 &&  wrappedAround) overflow = true; // Case 8

    return overflow;
}

bool CircularBuffer_read(CircularBuffer* circBuf, uint8_t* buffer, uint32_t length){

    bool overrun = false;      // Boolean indicating an overrun occurred
    bool wrappedAround = false;  // Boolean indicating if the readIndex has been reset to 0. Used for overrun detection

    // Used for overflow detection
    bool readAhead1 = circBuf->indexWrite < circBuf->indexRead;

    // More data requested than available in the buffer. Only read the last circBuf->bufferSize bytes of the data
    if(circBuf->bufferSize < length){
        // Set the number of bytes to read to circBuf->bufferSize
        length = circBuf->bufferSize;
        overrun = true;
    }

    // Check if reading the message would overflow the buffer. If so, wrapping is needed
    wrappedAround = circBuf->bufferSize < circBuf->indexRead + length;

    // Only read if the buffer is actually initialized
    if(!circBuf->onlyTrackIndex){
        if(wrappedAround){
            uint32_t bytesAtEnd = circBuf->bufferSize - circBuf->indexRead;
            uint32_t bytesAtBegin = length - bytesAtEnd;
            // Copy first part of the data to the end of the buffer
            memcpy(buffer, circBuf->buffer + circBuf->indexRead, bytesAtEnd);
            // Copy second part of the data to the beginning of the buffer
            memcpy(buffer + bytesAtEnd, circBuf->buffer, bytesAtBegin);
            wrappedAround = true;
        }else{
            memcpy(buffer, circBuf->buffer + circBuf->indexRead, length);
        }
    }

    /* Check if an overrun has happened
     * readAhead1 | readAhead2 | wrappedAround | interpretation | Example buffer drawing
     *1   false   |     false  |       false  | No overrun      ..R...W..... => ....R.W.....
     *2   false   |     false  |       true   | overrun         ..R...W..... => .R....W..... => enough bytes to pass Write and wrap around. Buffer possibly too small for bytes requested
     *3   false   |     true   |       false  | overrun         ..R...W..... => ......W.R... => enough bytes to pass Read
     *4   false   |     true   |       true   | overrun         circBuf->bufferSize < length. Buffer possibly too small for bytes requested
     *5   true    |     false  |       false  | Impossible      Read cannot move backwards without wrapping around
     *6   true    |     false  |       true   | No overrun      ......W.R... => ..R...W.....
     *7   true    |     true   |       false  | No overrun      ......W.R... => ......W...R.
     *8   true    |     true   |       true   | overrun         ......W...R. => ......W.R... => enough bytes requested to wrap around and pass Write.
    */

    // Update indexWrite
    circBuf->indexRead = (circBuf->indexRead + length) % circBuf->bufferSize;
    // Used for overrun detection
    bool readAhead2 = circBuf->indexWrite < circBuf->indexRead;

    if(!readAhead1 && !readAhead2 &&  wrappedAround) overrun = true; // Case 2
    if(!readAhead1 &&  readAhead2 && !wrappedAround) overrun = true; // Case 3
    if(!readAhead1 &&  readAhead2 && !wrappedAround) overrun = true; // Case 4
    if( readAhead1 &&  readAhead2 &&  wrappedAround) overrun = true; // Case 8

    return overrun;
}











/* ================================= Test functions below ================================= */
/*
void printCircBuf(CircularBuffer* circBuf){
    if(circBuf->onlyTrackIndex) return;

    uint32_t space = CircularBuffer_spaceFree(circBuf);
    printf("\nCircular buffer. Read=%u write=%u space=%u\n", circBuf->indexRead, circBuf->indexWrite, space);
    for(int i = 0; i < circBuf->bufferSize; i++){
        if(i == circBuf->indexWrite) printf("W");
        else printf(" ");
    }
    printf("\n");
    for(int i = 0; i < circBuf->bufferSize; i++){
        printf("%c", circBuf->buffer[i]);
    }
    printf("\n");
    for(int i = 0; i < circBuf->bufferSize; i++){
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
    bool ONLY_TRACK_INDEX = true;

    CircularBuffer* circBuf;

    uint8_t buffer[40];
    for(int i = 0; i < 40; i++){
        buffer[i] = i+65;
        printf("%c", buffer[i]);
    }
    printf("\n\n");
    bool overflow = false;

    // These tests have been written for a circular buffer with size 20;

    // case 1
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 5; circBuf->indexRead = 15;
    assert(circBuf->bufferSize == 20);
    printf("============================= Case 1 : write 5\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(CircularBuffer_canWrite(circBuf, 5));
    overflow = CircularBuffer_write(circBuf, buffer, 5);
    printCircBuf(circBuf);
    assert(!overflow);
    assert(circBuf->indexWrite == 10);
    assert(CircularBuffer_spaceFilled(circBuf) == 15);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 2
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 5; circBuf->indexRead = 15;
    printf("============================= Case 2 : write 17\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(!CircularBuffer_canWrite(circBuf, 17));
    overflow = CircularBuffer_write(circBuf, buffer, 17);
    printCircBuf(circBuf);
    assert(overflow);
    assert(circBuf->indexWrite == 2);
    assert(CircularBuffer_spaceFilled(circBuf) == 7);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 3
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 5; circBuf->indexRead = 15;
    printf("============================= Case 3 : write 12\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(!CircularBuffer_canWrite(circBuf, 12));
    overflow = CircularBuffer_write(circBuf, buffer, 12);
    printCircBuf(circBuf);
    assert(overflow);
    assert(circBuf->indexWrite == 17);
    assert(CircularBuffer_spaceFilled(circBuf) == 2);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 4
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 5; circBuf->indexRead = 10;
    printf("============================= Case 4 : write 30\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 4);
    assert(!CircularBuffer_canWrite(circBuf, circBuf->bufferSize + 10));
    overflow = CircularBuffer_write(circBuf, buffer, circBuf->bufferSize + 10);
    printCircBuf(circBuf);
    assert(overflow);
    assert(circBuf->indexWrite == 5);
    assert(CircularBuffer_spaceFilled(circBuf) == 15);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 5 => impossible

    // case 6
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 15; circBuf->indexRead = 10;
    printf("============================= Case 6 : write 10\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 14);
    assert(CircularBuffer_canWrite(circBuf, 10));
    overflow = CircularBuffer_write(circBuf, buffer, 10);
    printCircBuf(circBuf);
    assert(!overflow);
    assert(circBuf->indexWrite == 5);
    assert(CircularBuffer_spaceFilled(circBuf) == 15);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 7
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 15; circBuf->indexRead = 10;
    printf("============================= Case 7 : write 3\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 14);
    assert(CircularBuffer_canWrite(circBuf, 3));
    overflow = CircularBuffer_write(circBuf, buffer, 3);
    printCircBuf(circBuf);
    assert(!overflow);
    assert(circBuf->indexWrite == 18);
    assert(CircularBuffer_spaceFilled(circBuf) == 8);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 8
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexWrite = 15; circBuf->indexRead = 10;
    printf("============================= Case 8 : write 18\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 14);
    assert(!CircularBuffer_canWrite(circBuf, circBuf->bufferSize-2));
    overflow = CircularBuffer_write(circBuf, buffer, circBuf->bufferSize - 2);
    printCircBuf(circBuf);
    assert(overflow);
    assert(circBuf->indexWrite == 13);
    assert(CircularBuffer_spaceFilled(circBuf) == 3);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    printf("All write tests passed\n\n");
}

void testRead(){
    bool ONLY_TRACK_INDEX = true;

    CircularBuffer* circBuf;

    uint32_t bufferSize = 20;

    uint8_t buffer[bufferSize*2];
    uint8_t readBuffer[bufferSize*2];
    for(int i = 0; i < bufferSize*2; i++) {
        buffer[i] = i + 65;
        readBuffer[i] = '.';
    }

    bool overrun = false;

    // case 1
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 5; circBuf->indexWrite = 15;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 1 : read 5\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(CircularBuffer_canRead(circBuf, 5));
    overrun = CircularBuffer_read(circBuf, readBuffer, 5);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 5);
    assert(!overrun);
    assert(circBuf->indexRead == 10);
    assert(CircularBuffer_spaceFilled(circBuf) == 5);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 2
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 5; circBuf->indexWrite = 15;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 2 : read 17\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(!CircularBuffer_canRead(circBuf, 17));
    overrun = CircularBuffer_read(circBuf, readBuffer, 17);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, circBuf->bufferSize - 3);
    assert(overrun);
    assert(circBuf->indexRead == 2);
    assert(CircularBuffer_spaceFilled(circBuf) == 13);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 3
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 5; circBuf->indexWrite = 15;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 3 : read 12\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 9);
    assert(!CircularBuffer_canRead(circBuf, 12));
    overrun = CircularBuffer_read(circBuf, readBuffer, 12);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 12);
    assert(overrun);
    assert(circBuf->indexRead == 17);
    assert(CircularBuffer_spaceFilled(circBuf) == 18);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 4
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 5; circBuf->indexWrite = 10;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 4 : read 30\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 14);
    assert(!CircularBuffer_canRead(circBuf, 30));
    overrun = CircularBuffer_read(circBuf, readBuffer, 30);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 30);
    assert(overrun);
    assert(circBuf->indexRead == 5);
    assert(CircularBuffer_spaceFilled(circBuf) == 5);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 5 => impossible

    // case 6
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 15; circBuf->indexWrite = 10;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 6 : read 10\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 4);
    assert(CircularBuffer_canRead(circBuf, 10));
    overrun = CircularBuffer_read(circBuf, readBuffer, 10);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 10);
    assert(!overrun);
    assert(circBuf->indexRead == 5);
    assert(CircularBuffer_spaceFilled(circBuf) == 5);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 7
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 15; circBuf->indexWrite = 10;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 7 : read 3\n");
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 4);
    assert(CircularBuffer_canRead(circBuf, 3));
    overrun = CircularBuffer_read(circBuf, readBuffer, 3);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 3);
    assert(!overrun);
    assert(circBuf->indexRead == 18);
    assert(CircularBuffer_spaceFilled(circBuf) == 12);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);

    // case 8
    circBuf = CircularBuffer_init(ONLY_TRACK_INDEX, 20); circBuf->indexRead = 15; circBuf->indexWrite = 10;
    CircularBuffer_write(circBuf, buffer, circBuf->bufferSize);
    printf("============================= Case 8 : read 18\n");
    printAndResetBuf(readBuffer, 5);
    printCircBuf(circBuf);
    assert(CircularBuffer_spaceFree(circBuf) == 4);
    assert(!CircularBuffer_canRead(circBuf, 18));
    overrun = CircularBuffer_read(circBuf, readBuffer, 18);
    printCircBuf(circBuf);
    printAndResetBuf(readBuffer, 18);
    assert(overrun);
    assert(circBuf->indexRead == 13);
    assert(CircularBuffer_spaceFilled(circBuf) == 17);
    assert(CircularBuffer_spaceFree(circBuf) == circBuf->bufferSize - 1 - CircularBuffer_spaceFilled(circBuf));
    free(circBuf);
    printf("All read tests passed\n\n");
}

int main(int argc, char *argv[]){
    testWrite();
    testRead();
}
*/