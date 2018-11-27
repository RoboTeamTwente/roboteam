/*
 * packing.c
 *  Created on: Nov 27, 2018
 *      Author: rolf
 */

#include "packing.h"

//Convert bytes into a receivedData struct.
void packetToReceivedData(uint8_t input[RECEIVEPKTLEN], receivedData *output){
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
	output->id = input[0]>>3; //a
	output->rho = (input[0]&0b111)<<8; //b
	output->rho |= input[1]; //b
	output->theta = input[2]<<3; //c
	output->theta |= (input[3]>>5)&0b111; //c
	output->driving_reference = (input[3]>>4)&1; //d
	output->use_cam_info = (input[3]>>3)&1; //e
	output->use_angle	= (input[3]>>2)&1; //f
	output->angular_control = (input[3]&0b11) << 8; //g
	output->angular_control |= input[4]; //g
	output->debug_info = (input[5]>>3)&1; //h
	output->do_kick = (input[5]>>2)&1; //i
	output->do_chip = (input[5]>>1)&1; //j
	output->kick_chip_forced = input[5]&1; //k
	output->kick_chip_power = input[6]; //m
	output->velocity_dribbler = input[7]; //n
	output->geneva_drive_state = (input[8]>>5)&0b111; //p
	output->cam_position_x = (input[8]&0b11111)<<8; //q
	output->cam_position_x |= input[9]; //q
	output->cam_position_y = input[10] << 5; //r
	output->cam_position_y |= (input[11]>>3)&0b11111; //r
	output->cam_rotation = (input[11]&0b111) << 8; //s
	output->cam_rotation |= input[12]; //s
}

//Converts a receivedData into a robotData struct, doing unit and type conversions. needs to be tested.
void convertReceivedData(receivedData *input, robotData *output){
	output->id=input->id;
	output->driving_reference=input->driving_reference;
	output->use_cam_info=input->use_cam_info;
	output->use_angle=input->use_angle;
	output->debug_info=input->debug_info;
	output->do_kick=input->do_kick;
	output->do_chip=input->do_chip;
	output->kick_chip_forced=input->kick_chip_forced;


	output->geneva_drive_state=input->geneva_drive_state;								//[(0)1,5]->[-2,2]. Conversion is done in geneva file.
	output->rho=(float)input->rho									*0.004F; 			//[0, 2047]-> m/s [0,8.191]
	output->theta=(float)input->theta								/1024.0F * M_PI;	//[-1024,1023]-> rad [-pi,pi]
	output->kick_chip_power=(float)input->kick_chip_power			/255.0F*100.0F;		//[0,255]-> %[0,100]  8.0F was used instead of 100.0F in the past?? conflicting comments in robothub
	output->velocity_dribbler=(float)input->velocity_dribbler		/255.0F*100.0F;		//[0,255]-> %[0,100]  8.0F was used instead of 100.0F in the past?? conflicting comments in robothub

	// These are changed/used based on the booleans above
	if (input->use_angle){
		output->angular_control=(float)input->angular_control		/512.0F*M_PI;		//[-512,511]-> rad [-pi,pi]
	}else{
		output->angular_control=(float)input->angular_control		/512.0F*16.0F*M_PI;	//[-512,511]-> rad/s [-8*2pi,8*2pi]
	}
	//only used if use_cam_info is true.
	output->cam_position_x=(float)input->cam_position_x				/400.0F;			//[-4096,4095]-> m [-10.24,10.23]
	output->cam_position_y=(float)input->cam_position_y				/400.0F;			//[-4096,4095]-> m [-10.24,10.23]
	output->cam_rotation=(float)input->cam_rotation					/1024.0F*M_PI;		//[-1024,1023]-> rad [-pi,pi]
}
//Converts robot feedback into bytes to send
void feedbackDataToPacket(feedbackData *input, uint8_t output[FULLFBPKTLEN]) {
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

float uint32tofloat(uint32_t raw) {

	union float_bytes {
	       float val;
	       uint32_t bytes;
	    } data;

	data.bytes = raw;
	return data.val;

}

void printReceivedData(receivedData *input, uint8_t dataArray[RECEIVEPKTLEN]) {
	uprintf("----->FROM BASESTATION----->\n");
	for(int i=0; i<RECEIVEPKTLEN; i++) {
		uprintf("%02x ", dataArray[i]);
	}
	uprintf("\n");

	uprintf("\tRoboID: %i \n", input->id);
	uprintf("\tDebug info: %i \n", input->debug_info);
	uprintf("\tRho: %i \n\tTheta: %i \n", input->rho, input->theta);
	uprintf("\tKICKCHIP\n");
	uprintf("\tKick %i \n\t Chip: %i \n\t Forced: %i \n\t Power: %i \n", input->do_kick, input->do_chip ,input->kick_chip_forced, input->kick_chip_power);
	uprintf("\tDribbler velocity: %i \n", input->velocity_dribbler);
	uprintf("\tGeneva drive: %i \n", input->geneva_drive_state);
	uprintf("\tDriving reference: %i \n", input->driving_reference);
	if(input->use_angle){
		uprintf("\tAngular velocity: %i \n", input->angular_control);
	}
	else{
		uprintf("\tAngle: %i \n", input->angular_control);
	}
	uprintf("\tCAMERA \n\t use cam info: %i \n\t position x: %i \n\t position y: %i \n\t rotation: %i \n\n", input->use_cam_info, input->cam_position_x, input->cam_position_y, input->cam_rotation);
}

void printRoboAckData(feedbackData *input, uint8_t dataArray[32], uint8_t feedbackDataLength) {
	uprintf("<-----TO BASESTATION<-----\n");

	//print feedback packet in hex
	for(int i=0; i<feedbackDataLength; i++) {
		uprintf("%02x ", dataArray[i]);
	}
	uprintf("\n");

	uprintf("\tRoboID: %i \n", input->roboID);
	uprintf("\tWHEELS \n\t leftFront: %i \n\t rightFront: %i \n\t leftBack: %i \n\t rightBack: %i \n", input->wheelLeftFront, input->wheelRightFront, input->wheelLeftBack, input->wheelRightBack);
	uprintf("\tGeneva drive: %i \n", input->genevaDriveState);
	uprintf("\tBattery: %i \n", input->batteryState);
	uprintf("\tPOSITION \n\t x: %i \n\t y: %i \n", input->xPosRobot, input->yPosRobot);
	uprintf("\tRho: %i \n\tTheta: %i \n", input->rho, input->theta);
	uprintf("\tOrientation: %i \n", input->orientation);
	uprintf("\tAngular velocity: %i \n", input->angularVelocity);
	uprintf("\tBall sensor: %i \n", input->ballSensor);
	uprintf("\tXSENS \n\t x: %.6f \n\t y: %.6f \n\t w: %.6f\n\n", uint32tofloat(input->xAcceleration), uint32tofloat(input->yAcceleration), uint32tofloat(input->angularRate));
}
