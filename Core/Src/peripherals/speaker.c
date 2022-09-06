#include "speaker.h"
#include "peripheral_util.h"
#include "logging.h"

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Send a command to the speaker
static void sendCommand(uint8_t* command, uint8_t length);




///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void speaker_Play(){
    uint8_t command[4] = {0x7e, 0x02, 0x01, 0xef};
    sendCommand(command, 4);
}
void speaker_Pause(){
    uint8_t command[4] = {0x7e, 0x02, 0x02, 0xef};
    sendCommand(command, 4);
}
void speaker_NextSong(){
    uint8_t command[4] = {0x7e, 0x02, 0x03, 0xef};
    sendCommand(command, 4);
}
void speaker_PreviousSong(){
    uint8_t command[4] = {0x7e, 0x02, 0x04, 0xef};
    sendCommand(command, 4);
}
void speaker_VolumeUp(){
    uint8_t command[4] = {0x7e, 0x02, 0x05, 0xef};
    sendCommand(command, 4);
}
void speaker_VolumeDown(){
    uint8_t command[4] = {0x7e, 0x02, 0x06, 0xef};
    sendCommand(command, 4);
}
void speaker_Stop(){
    uint8_t command[4] = {0x7e, 0x02, 0x0e, 0xef};
    sendCommand(command, 4);
}





void speaker_SelectSong(uint8_t folder, uint8_t song){
    uint8_t command[6] = {0x7e, 0x04, 0x42, folder, song, 0xef};
    sendCommand(command, 6);
}

void speaker_Setvolume(uint8_t level){
    level = level & 0b11111; // Cap volume level between 0 and 31, as per the datasheet
    uint8_t command[5] = {0x7e, 0x03, 0x31, level, 0xef};
    sendCommand(command, 5);
    
}

void speaker_HandleCommand(REM_RobotMusicCommand* rmc){
    LOG("speaker_HandleCommand()\n");

    /* Volume */
    // Set the volume if it's not 0
    if(0 < rmc->volume) speaker_Setvolume(rmc->volume);
    if(rmc->volumeUp) speaker_VolumeUp();
    if(rmc->volumeDown) speaker_VolumeDown();
    /* Song */
    // Select the right song only if one is given
    if(rmc->folderId != 0 && rmc->songId != 0) {
        speaker_SelectSong(rmc->folderId, rmc->songId);
        speaker_Play();
    }
    if(rmc->nextSong) speaker_NextSong();
    if(rmc->previousSong) speaker_PreviousSong();
    /* Mode */
    if(rmc->stop) speaker_Stop();
    if(rmc->pause) speaker_Pause();
    if(rmc->play) speaker_Play();

}




///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

/**
 * @brief Send a command to the speaker. 
 * 
 * @param command The command (byte array) to send to the speaker
 * @param length The length of the command (number of bytes)
 */
static void sendCommand(uint8_t* command, uint8_t length){
    HAL_UART_Transmit(UART_BACK, command, length, 10);
    HAL_Delay(10); // Delay needed. Speaker can't handle two command right after another
}