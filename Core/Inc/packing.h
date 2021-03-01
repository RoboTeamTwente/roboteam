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

#define ROBOPKTLEN 8 //amount of bytes for a packet sent to the robot
#define SHORTACKPKTLEN 11 //amount of bytes of an ACK packet sent by the robot without using the extra/debug fields
#define FULLACKPKTLEN 8 //ACK packet with debug fields

// Conversion constants
#define CONVERT_RHO 			0.004f
#define CONVERT_THETA 			0.00307f
#define CONVERT_YAW_REF 		0.00614f
#define CONVERT_SHOOTING_POWER 	0.39f
#define CONVERT_DRIBBLE_SPEED 	14.285f
#define CONVERT_VISION_YAW 		0.00307f

///////////////////////////////////////////////////// STRUCTS

typedef struct ReceivedData {
	float stateRef[3];
	bool visionAvailable;
	float visionYaw;
	int genevaRef;
	int dribblerRef;
	int shootPower;
	bool kick_chip_forced;
	bool do_kick;
	bool do_chip;
} ReceivedData;

//for debugging
float uint32tofloat(uint32_t raw);

void packetToRoboData(RobotCommandPayload* input, ReceivedData* receivedData);

#endif /* PACKING_H_ */

