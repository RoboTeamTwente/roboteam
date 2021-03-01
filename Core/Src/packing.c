/*
 * packing.c
 *
 *  Created on: Mar 27, 2017
 *      Author: gebruiker
 */

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')


#include "packing.h"
#include <stdio.h>
#include "PuTTY.h" // apparently only for debugging

#include "RobotCommand.h"
#include "RobotFeedback.h"
#include "peripheral_util.h"

/*
 * Create a roboData structure from a given Bytearray.
 * This is used by the robot to convert a received nRF packet into a struct with named variables.
 */

void packetToRoboData(RobotCommandPayload* input, ReceivedData* receivedData) {
	RobotCommand rc = {0};
	

	decodeRobotCommand(&rc, input);

	/*
	 * Convert data to useful units
	 */
	// State
	//static float stateRef[3] = {0, 0, 0};
	receivedData->stateRef[body_x] = (rc.rho) * cosf(rc.theta);
	receivedData->stateRef[body_y] = (rc.rho) * sinf(rc.theta);
	receivedData->stateRef[body_w] = rc.angle;
	//receivedData->stateRef = stateRef;

	// Geneva
	// receivedData->genevaRef = rc.geneva;

	// Dribbler
	receivedData->dribblerRef = rc.dribbler * 14.285f; // Scale [0, 7] to [0, 100]

	// Shoot
	// TODO fix this rounding of shootPower
	receivedData->shootPower = rc.kickChipPower * 14.285f; // Scale [0, 7] to [0, 100]
	receivedData->kick_chip_forced = rc.doForce;
	receivedData->do_kick = rc.doKick;
	receivedData->do_chip = rc.doChip;

	// Vision data
	receivedData->visionAvailable = rc.useCameraAngle;
	receivedData->visionYaw = rc.cameraAngle; // TODO check if this should be converted to something else
}
