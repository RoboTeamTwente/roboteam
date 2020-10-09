#include <stdbool.h>


#ifndef __MSG_BUFF_STATUS_H
#define __MSG_BUFF_STATUS_H

bool isTransmitting;


/*
 * create a struct that keeps track of all buffered messages and if they are new or not.
 *  when it is send isNew should be set to false.
 */
struct msgsBufferStatus {
	uint8_t command[PACKET_SIZE_ROBOT_COMMAND]; // packet size in bytes
	bool isNewCommand;
	uint8_t feedback[PACKET_SIZE_ROBOT_FEEDBACK];
	bool isNewFeedback;
	uint8_t packetsSent;
	uint8_t packetsReceived;
};

struct msgsBufferStatus msgBuff[16];

uint8_t BS_DEBUG;


#endif // __MSG_BUFF_STATUS_H
