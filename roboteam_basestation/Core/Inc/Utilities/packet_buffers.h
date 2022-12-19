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
#include "REM_RobotSetPIDGains.h"
#include "REM_RobotPIDGains.h"
#include "REM_RobotMusicCommand.h"

REM_SX1280FillerPayload SX1280_filler_payload;

#include "CircularBuffer.h"

typedef struct _wrapper_REM_RobotCommand {
	REM_RobotCommandPayload packet;
	bool isNewPacket;
} wrapper_REM_RobotCommand;

typedef struct _wrapper_REM_RobotFeedback {
	REM_RobotFeedbackPayload packet;
	bool isNewPacket;
} wrapper_REM_RobotFeedback;



wrapper_REM_RobotCommand buffer_REM_RobotCommand[MAX_NUMBER_OF_ROBOTS];
wrapper_REM_RobotFeedback buffer_REM_RobotFeedback[MAX_NUMBER_OF_ROBOTS];

CircularBuffer* nonpriority_queue_robots_index[MAX_NUMBER_OF_ROBOTS];
CircularBuffer* nonpriority_queue_pc_index;
CircularBuffer* nonpriority_queue_bs_index;

typedef struct _Wrapper_REM_Packet {
  uint8_t data[REM_MAX_TOTAL_PACKET_SIZE_SX1280];
} Wrapper_REM_Packet;

Wrapper_REM_Packet nonpriority_queue_robots[MAX_NUMBER_OF_ROBOTS][40];
Wrapper_REM_Packet nonpriority_queue_pc[40];
Wrapper_REM_Packet nonpriority_queue_bs[40];



// uint8_t nonpriority_queue_robots[MAX_NUMBER_OF_ROBOTS][40][REM_MAX_TOTAL_PACKET_SIZE_SX1280];
// uint8_t nonpriority_queue_pc[40][REM_MAX_TOTAL_PACKET_SIZE_SX1280];
// uint8_t nonpriority_queue_bs[40][REM_MAX_TOTAL_PACKET_SIZE_SX1280];


#endif // __MSG_BUFF_STATUS_H
