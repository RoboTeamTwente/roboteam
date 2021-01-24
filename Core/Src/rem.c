#include <stdio.h>
#include <string.h>

#include "rem.h"
#include "BaseTypes.h"
#include "peripheral_util.h"
#include "robot.h"
#include "PuTTY.h"

// Buffer to move received packets in to
uint8_t REM_buffer[100];

/**
 * @brief Starts the first UART read. This read will eventually lead to 
 * REM_UARTCallback, which will schedule any subsequent reads. 
 * 
 * @param huart The UART to start the first read on. Generally UART_PC.
 */
void REM_UARTinit(UART_HandleTypeDef *huart){
    HAL_UART_Receive_IT(huart, REM_buffer, 1);
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
    if(packetType == PACKET_TYPE_ROBOT_COMMAND){
        // Receive the entire RobotCommand packet into REM_buffer, excluding the header byte
        HAL_UART_Receive(huart, REM_buffer+1, PACKET_SIZE_ROBOT_COMMAND-1, 100);
        // Hack. Store into a global, where robot.c can use it
        memcpy(&myRobotCommandPayload.payload, REM_buffer, PACKET_SIZE_ROBOT_COMMAND);
        decodeRobotCommand(&myRobotCommand, &myRobotCommandPayload);
        // Hack. Set flag for robot.c
        robotCommandIsFresh = 1;
    }else{
        // TODO add some error handling here or something.
        Putty_printf("Received unknown header\n");
    }
    
    // Schedule the read for the next header byte
    HAL_UART_Receive_IT(huart, REM_buffer, 1);

}