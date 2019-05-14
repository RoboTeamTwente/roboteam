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

void printRoboAckData(roboAckData *input, uint8_t dataArray[32], uint8_t ackDataLength) {
	Putty_printf("<-----TO BASESTATION<-----\n");

	//print ack packet in hex
	for(int i=0; i<ackDataLength; i++) {
		Putty_printf("%02x ", dataArray[i]);
	}
	Putty_printf("\n");

	Putty_printf("\tRoboID: %i \n", input->roboID);
	Putty_printf("\tWHEELS \n\t leftFront: %i \n\t rightFront: %i \n\t leftBack: %i \n\t rightBack: %i \n", input->wheelLeftFront, input->wheelRightFront, input->wheelLeftBack, input->wheelRightBack);
	Putty_printf("\tGeneva drive: %i \n", input->genevaDriveState);
	Putty_printf("\tBattery: %i \n", input->batteryState);
	Putty_printf("\tPOSITION \n\t x: %i \n\t y: %i \n", input->xPosRobot, input->yPosRobot);
	Putty_printf("\tRho: %i \n\tTheta: %i \n", input->rho, input->theta);
	Putty_printf("\tOrientation: %i \n", input->orientation);
	Putty_printf("\tAngular velocity: %i \n", input->angularVelocity);
	Putty_printf("\tBall sensor: %i \n", input->ballSensor);
	Putty_printf("\tXSENS \n\t x: %.6f \n\t y: %.6f \n\t w: %.6f\n\n", uint32tofloat(input->xAcceleration), uint32tofloat(input->yAcceleration), uint32tofloat(input->angularRate));
}


/*
 * Create a roboData structure from a given Bytearray.
 * This is used by the robot to convert a received nRF packet into a struct with named variables.
 */
void packetToRoboData(uint8_t input[ROBOPKTLEN], ReceivedData* receivedData) {
	/*
	output[0] aaaaabbb
	output[1] bbbbbbbb
	output[2] cccccccc
	output[3] cccdefgg
	output[4] gggggggg
	output[5] 0000hijk
	output[6] mmmmmmmm
	output[7] nnnnnnnn
	output[8] pppqqqqq
	output[9] qqqqqqqq
	output[10] rrrrrrrr
	output[11] rrrrrsss
	output[12] ssssssss
	 */


//	Putty_printf("ptrd: "BYTE_TO_BINARY_PATTERN" "BYTE_TO_BINARY_PATTERN"\r\n", BYTE_TO_BINARY(input[3]), BYTE_TO_BINARY(input[4]));

	/*
	 * Read data from packet
	 */
	uint16_t rho = (input[0]) << 3;
	rho |= (input[1] >> 5) & 0b00000111;

	int16_t theta = (input[1] & 0b00011111) << 6;
	theta |= (input[2] >> 2) & 0b00111111;

	int16_t velocity_angular = (input[2] & 0b00000011) << 8;
	velocity_angular |= input[3] & 0b11111111;

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
	receivedData->do_kick = do_kick;
	receivedData->do_chip = do_chip;

	// Vision data
	receivedData->visionAvailable = use_cam_info;
	receivedData->visionYaw = cam_rotation * CONVERT_VISION_YAW;
}


/*
 * For the Robot ACKs we use the following packet definition
 *
 *skipping some characters for better readability

		Character   Description                 Values          Represented values    Units       Bits    Comment
		a           Robot ID                    [0,31]          [0,31]                -              5    -
		b           Left front wheel state      [0,1]           {true,false}          -              1    Indicates whether the left front wheel functions
		c           Right front wheel state     [0,1]           {true,false}          -              1    Indicates whether the right front wheel functions
		d           Left back wheel state       [0,1]           {true,false}          -              1    Indicates whether the left back wheel functions
		e           Right back wheel state      [0,1]           {true,false}          -              1    Indicates whether the right back wheel functions
		f           Geneva drive state          [0,1]           {true,false}          -              1    Indicates whether the Geneva drive functions
		g           Battery state               [0,1]           {true,false}          -              1    States whether the battery is nearing depletion
		h           x position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		k           y position robot            [-4096,4095]    [-1024,1023]          0.25cm        13    -
		m           rho            				[-1024,1023]    [?,?]		          	            11    Magnitude of the robot velocity vector
		o           theta           			[-1024,1023]    [?,?]                               11    Angle of the robot velocity vector
		p           Orientation                 [-1024,1023]    [-pi,pi>              0.00307rad    11    Angle of the facing direction. 2048 possible angles. Intervals of ~0.00307 rad
		q           Angular velocity            [-1024,1023]    [?,?]                 0.049rad/s?   11
		s           Ball sensor                 [0,128]         {?}			          -              7    Can be used to specify where on the dribbler the ball is located. For now a non-zero value represents the presence of the ball

		Extra
		t           Acceleration x              [0, 4294967295]    [0, 32 bit float]       m/s/s         32    -
		u           Acceleration y              [0, 4294967295]    [0, 32 bit float]       m/s/s         32    -
		v           Angular rate                [0, 4294967295]    [0, 32 bit float]       m/s/s         32    raw angular velocity from xsense


	===== Packet received from the robot =====
		Byte      Config
		 0        aaaaabcd
		 1        efghhhhh
		 2        hhhhhhhh
		 3        kkkkkkkk
		 4        kkkkkmmm
		 5        mmmmmmmm
		 6        oooooooo
		 7        oooppppp
		 8        ppppppqq
		 9        qqqqqqqq
		10        qsssssss

		Extra
		11        tttttttt
		12        tttttttt
		13        tttttttt
		14        tttttttt
		15        uuuuuuuu
		16        uuuuuuuu
		17        uuuuuuuu
		18        uuuuuuuu
		19        vvvvvvvv
		20        vvvvvvvv
		21        vvvvvvvv
		22        vvvvvvvv

 */




/*
 * First, fill the fields on a roboAckData struct.
 * Then convert that struct into a Bytearray by using this function.
 * The result can be used as an ACK payload to transmit it over air.
 */

void roboAckDataToPacket(roboAckData *input, uint8_t output[FULLACKPKTLEN]) {
	output[0]  = (uint8_t) ((input->roboID)<<3); //a
	output[0] |= (uint8_t) ((input->wheelLeftFront)<<2); //b
	output[0] |= (uint8_t) ((input->wheelRightFront)<<1); //c
	output[0] |= (uint8_t) ((input->wheelLeftBack)); //d


	output[1]  = (uint8_t) ((input->wheelRightBack)<<7); //e
	output[1] |= (uint8_t) ((input->genevaDriveState)<<6); //f
	output[1] |= (uint8_t) ((input->batteryState)<<5); //g
	output[1] |= (uint8_t) ((input->xPosRobot>>8))&0b11111; //h

	output[2]  = (uint8_t) (input->xPosRobot&0xff); //h

	output[3]  = (uint8_t) (input->yPosRobot>>5)&0xff; //k

	output[4]  = (uint8_t) ((input->yPosRobot&0b11111)<<3); //k
	output[4] |= (uint8_t) (input->rho>>8)&0b111; //m

	output[5]  = (uint8_t) (input->rho&0xff); //m

	output[6]  = (uint8_t) ((input->theta>>3)&0xff); //o

	output[7]   = (uint8_t) ((input->theta&0xff)<<5); //o
	output[7]  |= (uint8_t) (input->orientation>>6)&0b11111; //p

	output[8]   = (uint8_t) (input->orientation&0b111111)<<2; //p
	output[8]  |= (uint8_t) (input->angularVelocity>>9)&0b11; //q

	output[9]  = (uint8_t) ((input->angularVelocity>>1)&0xff); //q

	output[10]  = (uint8_t) ((input->angularVelocity&1)<<7); //q

	output[10] |= (uint8_t) (input->ballSensor)&0x7f; //s

	output[11] = (uint8_t) (input->xAcceleration >> 24)&0xff; //t
	output[12] = (uint8_t) (input->xAcceleration >> 16)&0xff; //t
	output[13] = (uint8_t) (input->xAcceleration >> 8)&0xff; //t
	output[14] = (uint8_t) (input->xAcceleration)&0xff; //t

	output[15] = (uint8_t) (input->yAcceleration >> 24)&0xff; //u
	output[16] = (uint8_t) (input->yAcceleration >> 16)&0xff; //u
	output[17] = (uint8_t) (input->yAcceleration >> 8)&0xff; //u
	output[18] = (uint8_t) (input->yAcceleration)&0xff; //u

	output[19] = (uint8_t) (input->angularRate >> 24)&0xff; //v
	output[20] = (uint8_t) (input->angularRate >> 16)&0xff; //v
	output[21] = (uint8_t) (input->angularRate >> 8)&0xff; //v
	output[22] = (uint8_t) (input->angularRate)&0xff; //v

}

/*
 * We would actually just pass the raw ack packet to the computer and let the computer handle the unwrapping.
 * But for completeness and for debug purposes it is nice to unwrap the ACK packet on the Basestation board
 * itself. In that way we can test and debug with just the Basestation and a serial monitor.
 *
 * ACK packets can be either 11 Bytes or 23 Bytes long. That depends on whether the Basestation requested
 * additional fields buy setting the debug_info flag in an earlier robo packet.
 */
void ackPacketToRoboAckData(uint8_t input[23], uint8_t packetlength, roboAckData *output) {
	//input is now specified as an array of size 23. Note that there are also ACK packets of the length 11.
	//You need to use packetlength to know which range of the array contains useful information.
	//The attempt of accessing input[11] to input[22] for a short ACK packet will yield garbage data.

	output->roboID = input[0]>>3; //a
	output->wheelLeftFront = (input[0]>>2)&1; //b
	output->wheelRightFront = (input[0]>>1)&1; //c
	output->wheelLeftBack = (input[0])&1; //d

	output->wheelRightBack = (input[1]>>7)&1; //e
	output->genevaDriveState = (input[1]>>6)&1; //f
	output->batteryState = (input[1]>>5)&1;  //g
	output->xPosRobot = ((input[1]&0b11111)<<8 | input[2]); //h

	output->yPosRobot = ((input[3]<<5) | (input[4]>>3)); //k

	output->rho = (input[4]&0b111)<<8; //m

	output->rho |= input[5]; //m

	output->theta = (input[6]<<3) | (input[7]>>5); //o

	output->orientation = ((input[7]&0b11111)<<6) | (input[8]>>2); //p

	output->angularVelocity = ((input[8]&0b11)<<9) | (input[9]<<1) | ((input[10]>>7)&1); //q

	output->ballSensor = input[10]&0x7f; //s

	if(packetlength < FULLACKPKTLEN)
		return;

	//extra data
	output->xAcceleration = (input[11]<<24) | (input[12]<<16) | (input[13]<<8) | input[14]; //t
	output->yAcceleration = (input[15]<<24) | (input[16]<<16) | (input[17]<<8) | input[18]; //u
	output->angularRate = (input[19]<<24) | (input[20]<<16) | (input[21]<<8) | input[22]; //v

}
