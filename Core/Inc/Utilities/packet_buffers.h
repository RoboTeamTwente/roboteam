
#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

#include <stdbool.h>
#include "BaseTypes.h"
#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "RobotBuzzer.h"

bool isTransmitting;


/*
 * create a struct that keeps track of all buffered messages and if they are new or not.
 *  when it is send isNew should be set to false.
 */
// struct msgsBufferStatus {
// 	RobotCommandPayload command;
// 	bool isNewCommand;
// 	RobotFeedbackPayload feedback;
// 	bool isNewFeedback;
// 	uint8_t packetsSent;
// 	uint8_t packetsReceived;
// };
// struct msgsBufferStatus msgBuff[16];

// struct msgBufferRobotBuzzer {
// 	RobotBuzzerPayload command;
// 	bool isNewCommand;
// };
// struct msgBufferRobotBuzzer msgBuffRobotBuzzer[16];


struct _buffer_RobotCommand {
	RobotCommandPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotCommand buffer_RobotCommand[16];


struct _buffer_RobotFeedback {
	RobotFeedbackPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotFeedback buffer_RobotFeedback[16];


typedef struct _buffer_RobotBuzzer {
	RobotBuzzerPayload packet;
	bool isNewPacket;
	uint32_t counter;
};
struct _buffer_RobotBuzzer buffer_RobotBuzzer[16];



#endif // __MSG_BUFF_STATUS_H
