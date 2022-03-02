
#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

#include <stdbool.h>
#include "basestation.h"

#include "REM_BaseTypes.h"
#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotBuzzer.h"
#include "REM_RobotStateInfo.h"

bool isTransmitting;

struct _buffer_RobotCommand {
	REM_RobotCommandPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotCommand buffer_RobotCommand[MAX_NUMBER_OF_ROBOTS];


struct _buffer_RobotFeedback {
	REM_RobotFeedbackPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotFeedback buffer_RobotFeedback[MAX_NUMBER_OF_ROBOTS];

struct _buffer_RobotStateInfo {
	REM_RobotStateInfoPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotStateInfo buffer_RobotStateInfo[MAX_NUMBER_OF_ROBOTS];

struct _buffer_RobotBuzzer {
	REM_RobotBuzzerPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotBuzzer buffer_RobotBuzzer[MAX_NUMBER_OF_ROBOTS];



#endif // __MSG_BUFF_STATUS_H
