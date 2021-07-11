
#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

#include <stdbool.h>
#include "basestation.h"

#include "BaseTypes.h"
#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "RobotBuzzer.h"
#include "RobotStateInfo.h"

bool isTransmitting;

struct _buffer_RobotCommand {
	RobotCommandPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotCommand buffer_RobotCommand[MAX_NUMBER_OF_ROBOTS];


struct _buffer_RobotFeedback {
	RobotFeedbackPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotFeedback buffer_RobotFeedback[MAX_NUMBER_OF_ROBOTS];

struct _buffer_RobotStateInfo {
	RobotStateInfoPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotStateInfo buffer_RobotStateInfo[MAX_NUMBER_OF_ROBOTS];

struct _buffer_RobotBuzzer {
	RobotBuzzerPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotBuzzer buffer_RobotBuzzer[MAX_NUMBER_OF_ROBOTS];



#endif // __MSG_BUFF_STATUS_H
