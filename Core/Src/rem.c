#include <stdio.h>
#include <string.h>

#include "rem.h"
#include "REM_BaseTypes.h"
#include "REM_RobotCommand.h"
#include "REM_RobotMusicCommand.h"
#include "peripheral_util.h"
#include "gpio_util.h"
#include "robot.h"
#include "logging.h"
#include "main.h"

// Buffers to move received packets in to
static uint8_t REM_buffer[100];
static REM_RobotCommandPayload rcp;
static REM_RobotMusicCommandPayload rmcp;

/**
 * @brief Starts the first UART read. This read will eventually lead to 
 * REM_UARTCallback, which will schedule any subsequent reads. 
 * 
 * @param huart The UART to start the first read on. Generally UART_PC.
 */
void REM_UARTinit(UART_HandleTypeDef *huart){
    LOG("[REM_UARTinit]\n");
    HAL_UART_Receive_IT(huart, REM_buffer, 1); // if uncommented, only use with programmer connected
}

/**
 * @brief This function is called when a header byte is sent to the robot.
 * The length of the packet is determinted by this header byte, and a blocking
 * read waits for this number of bytes. After that, it schedules the read for
 * the next header byte.
 * 
 * TODO make the read for the packet bytes nonblocking
 * TODO move this to a more general place / add mure specific functions
 * TODO implement callbacks in robot.c or somewhere, to deal with the incoming
 * packets.
 * 
 * @param huart The UART to receive the bytes on. Generally UART_PC. 
 */
void REM_UARTCallback(UART_HandleTypeDef *huart){
    // Read the header byte
    uint8_t packetType = REM_buffer[0];
    // If RobotCommand
    if(packetType == REM_PACKET_TYPE_REM_ROBOT_COMMAND){
        // Receive the entire RobotCommand packet into REM_buffer, excluding the header byte
        HAL_UART_Receive(huart, REM_buffer+1, REM_PACKET_SIZE_REM_ROBOT_COMMAND-1, 100);
        // Store received packet in local RobotCommandPayload. Send to robot.c for decoding
        memcpy(&rcp.payload, REM_buffer, REM_PACKET_SIZE_REM_ROBOT_COMMAND);
        robot_setRobotCommandPayload(&rcp);
    }else
    if(packetType == REM_PACKET_TYPE_REM_ROBOT_MUSIC_COMMAND){
        HAL_UART_Receive(huart, REM_buffer+1, REM_PACKET_SIZE_REM_ROBOT_MUSIC_COMMAND-1, 100);
        memcpy(&rmcp.payload, REM_buffer, REM_PACKET_SIZE_REM_ROBOT_MUSIC_COMMAND);
        robot_setRobotMusicCommandPayload(&rmcp);
    }else{
        // TODO add some error handling here or something.
        LOG("Unknown header\n");
        // sprintf(logBuffer, "Received unknown header %d\n", packetType);
    }
    
    // Schedule the read for the next header byte
    HAL_UART_Receive_IT(huart, REM_buffer, 1);
    toggle_Pin(LED6_pin);

}