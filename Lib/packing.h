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

#define ROBOPKTLEN 13 //amount of bytes for a packet sent to the robot
#define SHORTACKPKTLEN 11 //amount of bytes of an ACK packet sent by the robot without using the extra/debug fields
#define FULLACKPKTLEN 23 //ACK packet with debug fields

/*
 * A data struct which is easy to work with
 * when accessing variables.
 * It needs to be converted before it can be
 * transmitted, though.
 */

									//Description                 Values            Units     	  Represented values    Bits
typedef struct roboData{
   uint8_t id:5;					//Robot ID                    [0, 15]           -             [0, 15]                  4
   uint16_t rho:11;					//Velocity length             [0, 2047]         0.004m/s      [0, 8.188]              11
   int16_t theta:11;				//Velocity angle              [-1024, 1023]     0.00307rad    [-pi, pi>               11
   uint8_t driving_reference:1;		//Driving reference           [0, 1]            -             {true, false}            1
   uint8_t use_cam_info:1;			//Use camera information      [0, 1]            -             {true, false}            1
   uint8_t use_angle:1;				//Go to the newly set angle	  [0, 1]			-			  {true, false}			   1
   int16_t velocity_angular:10;		//Reference angular velocity  [-512, 511]       0.098rad/s    [-8*2pi, 8*2pi]         10
   uint8_t debug_info:1;			//Debug information           [0, 1]            -             {true, false}            1
   uint8_t do_kick:1;				//Kick                        [0, 1]            -             {true, false}            1
   uint8_t do_chip:1;				//Chip                        [0, 1]            -             {true, false}            1
   uint8_t kick_chip_forced:1;		//Kick/chip immediately       [0, 1]            -             {true, false}            1
   uint8_t kick_chip_power:8;		//Kick/chip power             [0, 255]          0.39%         [0, 100]%                8
   uint8_t velocity_dribbler:8;		//Reference dribbler speed    [0, 255]          0.39%         [0, 100]%                8
   uint8_t geneva_drive_state:3;	//Geneva drive state          [0, 7]            -             [-2, 2]                  3
   int16_t cam_position_x:13;		//x position robot (camera)   [-4096, 4095]     0.0025m       [-10.24, 10.23]         13
   int16_t cam_position_y:13;		//y position robot (camera)   [-4096, 4095]     0.0025m       [-10.24, 10.23]         13
   int16_t cam_rotation:11;			//Orientation (camera)        [-1024, 1023]     0.00307rad    [-pi, pi]               11
} roboData;

//between 11 and 23 Bytes, ideally
typedef struct roboAckData{
	//regular fields: 11 Bytes
	uint8_t roboID:5;
	uint8_t wheelLeftFront:1;
	uint8_t wheelRightFront:1;
	uint8_t wheelLeftBack:1;
	uint8_t wheelRightBack:1;
	uint8_t genevaDriveState:1;
	uint8_t batteryState:1;
	int16_t xPosRobot:13;
	int16_t yPosRobot:13;
	int16_t rho:11;
	int16_t theta:11;
	int16_t orientation:11;
	int16_t angularVelocity:11;
	int8_t ballSensor:7;

	//extra fields (add 12 Bytes)
	uint32_t xAcceleration;
	uint32_t yAcceleration;
	uint32_t angularRate;

} roboAckData;


//for debugging
float uint32tofloat(uint32_t raw);
void printRoboData(roboData *input, uint8_t dataArray[ROBOPKTLEN]);
void printRoboAckData(roboAckData *input, uint8_t dataArray[32], uint8_t ackDataLength);

void robotDataToPacket(roboData *input, uint8_t output[ROBOPKTLEN]);
void packetToRoboData(uint8_t input[ROBOPKTLEN], roboData *output);
void roboAckDataToPacket(roboAckData *input, uint8_t output[FULLACKPKTLEN]);
void ackPacketToRoboAckData(uint8_t input[FULLACKPKTLEN], uint8_t packetlength, roboAckData *output);

#endif /* PACKING_H_ */

