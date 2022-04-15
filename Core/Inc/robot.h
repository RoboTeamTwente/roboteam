#ifndef __ROBOT__H_
#define __ROBOT__H_

#include <unistd.h>
#include "REM_RobotCommand.h"

// These three are currently a callback hack for the REM implementation
REM_RobotCommandPayload myRobotCommandPayload;
REM_RobotCommand myRobotCommand;
volatile uint8_t robotCommandIsFresh;

uint8_t ROBOT_ID;

void print(char _out[]);

void init(void);
void loop(void);

void robot_setRobotCommandPayload(REM_RobotCommandPayload* rcp);
bool handlePacket(uint8_t* packet_buffer, uint8_t packet_length);

#endif /* __ROBOT__H_ */