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

void printRoboData(roboData *input, uint8_t dataArray[ROBOPKTLEN]) {
	Putty_printf("----->FROM BASESTATION----->\n\r");
	for(int i=0; i<ROBOPKTLEN; i++) {
		Putty_printf("%02x ", dataArray[i]);
	}
	Putty_printf("\n\r");

	Putty_printf("\tRoboID: %i \n\r", input->id);
	Putty_printf("\tDebug info: %i \n\r", input->debug_info);
	Putty_printf("\tRho: %i \n\r\tTheta: %i \n\r", input->rho, input->theta);
	Putty_printf("\tKICKCHIP\n\r");
	Putty_printf("\tKick %i \n\r\t Chip: %i \n\r\t Forced: %i \n\r\t Power: %i \n\r", input->do_kick, input->do_chip ,input->kick_chip_forced, input->kick_chip_power);
	Putty_printf("\tDribbler velocity: %i \n\r", input->velocity_dribbler);
	Putty_printf("\tGeneva drive: %i \n\r", input->geneva_drive_state);
	Putty_printf("\tDriving reference: %i \n\r", input->driving_reference);
	Putty_printf("\tAngular velocity: %i \n\r", input->velocity_angular);
	Putty_printf("\tCAMERA \n\r\t use cam info: %i \n\r\t rotation: %i \n\r\n\r", input->use_cam_info, input->cam_rotation);
}


/*
 * Create a roboData structure from a given Bytearray.
 * This is used by the robot to convert a received nRF packet into a struct with named variables.
 */
void packetToRoboData(RobotCommandPayload input, ReceivedData* receivedData) {
	RobotCommand rc = {0};

	decodeRobotCommand(&rc, &input);

	/*
	 * Convert data to useful units
	 */
	// State
	//static float stateRef[3] = {0, 0, 0};
	receivedData->stateRef[body_x] = (rc.rho * CONVERT_RHO) * cosf(rc.theta * CONVERT_THETA);
	receivedData->stateRef[body_y] = (rc.rho * CONVERT_RHO) * sinf(rc.theta * CONVERT_THETA);
	receivedData->stateRef[body_w] = rc.angle * CONVERT_YAW_REF - M_PI;
	//receivedData->stateRef = stateRef;

	// Geneva
	// receivedData->genevaRef = rc.geneva;

	// Dribbler
	receivedData->dribblerRef = rc.dribbler * CONVERT_DRIBBLE_SPEED;

	// Shoot
	receivedData->shootPower = rc.kickChipPower * CONVERT_SHOOTING_POWER;
	receivedData->kick_chip_forced = rc.doForce;
	receivedData->do_kick = rc.doKick;
	receivedData->do_chip = rc.doChip;

	// Vision data
	receivedData->visionAvailable = rc.useCameraAngle;
	receivedData->visionYaw = rc.cameraAngle * CONVERT_VISION_YAW;
}
