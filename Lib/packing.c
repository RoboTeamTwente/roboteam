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

float uint32tofloat(uint32_t raw) {

	union float_bytes {
	       float val;
	       uint32_t bytes;
	    } data;

	data.bytes = raw;
	return data.val;

}

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
void packetToRoboData(uint8_t input[ROBOPKTLEN], ReceivedData* receivedData) {
	uint16_t rho = (input[0]) << 3;
	rho |= (input[1] >> 5) & 0b00000111;

	int16_t theta = (input[1] & 0b00011111) << 6;
	theta |= (input[2] >> 2) & 0b00111111;
	theta |= (theta & 0x0400) ? 0xF800 : 0x0000; // sign extension

	int16_t velocity_angular = (input[2] & 0b00000011) << 8;
	velocity_angular |= input[3] & 0b11111111;
	velocity_angular |= (velocity_angular & 0x0200) ? 0xFC00 : 0x0000; // sign extension

	uint8_t kick_chip_power = input[4] & 0b11111111;

	uint8_t do_kick = (input[5] & 0b10000000) >> 7;
	uint8_t do_chip = (input[5] & 0b01000000) >> 6;
	uint8_t kick_chip_forced = (input[5] & 0b00100000) >> 5;
	uint8_t debug_info = (input[5] & 0b00010000) >> 4;
	uint8_t use_cam_info = (input[5] & 0b00001000) >> 3;
	uint8_t geneva_drive_state = input[5] & 0b00000111;

	uint8_t velocity_dribbler = 0;
	velocity_dribbler = (input[6] & 0b11111000) >> 3;
	//velocity_dribbler = velocity_dribbler & 0b00011111;
	int16_t cam_rotation = (input[6] & 0b00000111) << 8; //s
	cam_rotation |= input[7] & 0b11111111; //s
	cam_rotation |= (cam_rotation & 0x0400) ? 0xF800 : 0x0000; // sign extension

	/*
	 * Convert data to useful units
	 */
	// State
	//static float stateRef[3] = {0, 0, 0};
	receivedData->stateRef[body_x] = (rho * CONVERT_RHO) * cosf(theta * CONVERT_THETA);
	receivedData->stateRef[body_y] = (rho * CONVERT_RHO) * sinf(theta * CONVERT_THETA);
	receivedData->stateRef[body_w] = velocity_angular * CONVERT_YAW_REF;
	//receivedData->stateRef = stateRef;

	// Geneva
	receivedData->genevaRef = geneva_drive_state;

	// Dribbler
	receivedData->dribblerRef = velocity_dribbler * CONVERT_DRIBBLE_SPEED;

	// Shoot
	receivedData->shootPower = kick_chip_power * CONVERT_SHOOTING_POWER;
	receivedData->kick_chip_forced = kick_chip_forced;
	receivedData->do_kick = do_kick;
	receivedData->do_chip = do_chip;

	// Vision data
	receivedData->visionAvailable = use_cam_info;
	receivedData->visionYaw = cam_rotation * CONVERT_VISION_YAW;
}


void roboAckDataToPacket(roboAckData *input, uint8_t output[ROBOPKTLEN]) {

	output[0]  = (input->roboID);

	output[1]  = (uint8_t) (input->XsensCalibrated << 7);
	output[1] |= (uint8_t) (input->battery << 6);
	output[1] |= (uint8_t) (input->ballSensorWorking << 5);
	output[1] |= (uint8_t) (input->ballPos & 0x0F);

	output[2]  = (uint8_t) (input->genevaWorking << 7);
	output[2]  = (uint8_t) (input->genevaState & 0x7F);

	output[3]  = (uint8_t) (input->rho >> 8);

	output[4]  = (uint8_t) ((input->rho & 0x03) << 5);
	output[4] |= (uint8_t) ((input->angle >> 5) & 0x1F);

	output[5]  = (uint8_t) ((input->angle & 0x1F) << 5);
	output[5]  = (uint8_t) ((input->theta >> 8) & 0x03);

	output[6]  = (uint8_t) (input->theta & 0xFF);

	output[7]  = (uint8_t) (input->wheelLocked << 7);
	output[7]  = (uint8_t) (input->signalStrength & 0x7F);
}

