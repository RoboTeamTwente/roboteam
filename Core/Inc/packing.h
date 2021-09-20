/*
 * packing.h
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 *      (Co-)Author: Ulf Stottmeister, April 2018
 */

#ifndef PACKING_H_
#define PACKING_H_

#include <inttypes.h>
#include <stdbool.h>
#include "control_util.h"
#include "RobotCommand.h"

typedef struct ReceivedData {
	float stateRef[3];
	bool visionAvailable;
	float visionYaw;
	int dribblerRef;
	int shootPower;
	bool kick_chip_forced;
	bool do_kick;
	bool do_chip;
} ReceivedData;

void packetToRoboData(RobotCommandPayload* input, ReceivedData* receivedData);

#endif /* PACKING_H_ */

