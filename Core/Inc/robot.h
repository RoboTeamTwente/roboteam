#ifndef __ROBOT__H_
#define __ROBOT__H_

#include <unistd.h>
#include "REM_RobotCommand.h"
#include "REM_RobotMusicCommand.h"

// These three are currently a callback hack for the REM implementation
REM_RobotCommandPayload myRobotCommandPayload;
REM_RobotCommand myRobotCommand;

void print(char _out[]);

void init(void);
void loop(void);
uint8_t robot_get_ID();
uint8_t robot_get_Channel();

void robot_setRobotCommandPayload(REM_RobotCommandPayload* rcp);
void robot_setRobotMusicCommandPayload(REM_RobotMusicCommandPayload* rmcp);
bool handlePacket(uint8_t* packet_buffer, uint8_t packet_length);

#endif /* __ROBOT__H_ */