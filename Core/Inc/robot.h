#ifndef __ROBOT__H_
#define __ROBOT__H_

#include <unistd.h>
#include "REM_RobotCommand.h"

// These three are currently a callback hack for the REM implementation
REM_RobotCommandPayload myRobotCommandPayload;
REM_RobotCommand myRobotCommand;
volatile uint8_t robotCommandIsFresh;

void print(char _out[]);

void init(void);
void loop(void);

#endif /* __ROBOT__H_ */