#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

#include <stdbool.h>
#include "basestation.h"

#include "REM_SX1280Filler.h"
#include "REM_BaseTypes.h"
#include "REM_RobotCommand.h"
#include "REM_RobotFeedback.h"
#include "REM_RobotBuzzer.h"
#include "REM_RobotStateInfo.h"
#include "REM_RobotGetPIDGains.h"
#include "REM_RobotPIDGains.h"

bool isTransmitting;
REM_SX1280FillerPayload SX1280_filler_payload;

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

struct _buffer_RobotGetPIDGains {
	REM_RobotGetPIDGainsPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotGetPIDGains buffer_RobotGetPIDGains[MAX_NUMBER_OF_ROBOTS];

struct _buffer_RobotPIDGains {
	REM_RobotPIDGainsPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotPIDGains buffer_RobotPIDGains[MAX_NUMBER_OF_ROBOTS];

#endif // __MSG_BUFF_STATUS_H
