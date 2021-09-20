#ifndef __ROBOT__H_
#define __ROBOT__H_

#include <unistd.h>
#include "RobotCommand.h"

// These three are currently a callback hack for the REM implementation
RobotCommandPayload myRobotCommandPayload;
RobotCommand myRobotCommand;
volatile uint8_t robotCommandIsFresh;

uint8_t ROBOT_ID;

void print(char _out[]);

void init(void);
void loop(void);

#endif /* __ROBOT__H_ */