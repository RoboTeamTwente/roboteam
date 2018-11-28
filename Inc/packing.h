/*
 * packing.h
 *
 *  Created on: Nov 27, 2018
 *      Author: rolf
 */

#ifndef PACKING_H_
#define PACKING_H_

#include <stdio.h>
#include <inttypes.h>
#include <math.h>

#define RECEIVEPKTLEN 13 //amount of bytes for a packet sent to the robot
#define SHORTFBPKTLEN 11 //amount of bytes of an feedback packet sent by the robot without using the extra/debug fields
#define FULLFBPKTLEN 23 //feedback packet with debug fields
typedef struct robotData{
	uint8_t id:5;
	float rho;
	float theta;
	uint8_t driving_reference;
	uint8_t use_cam_info;
	uint8_t use_angle;
	float angular_control;
	uint8_t debug_info;
	uint8_t do_kick;
	uint8_t do_chip;
	uint8_t kick_chip_forced;
	uint8_t kick_chip_power;
	float velocity_dribbler;
	uint8_t geneva_drive_state;
	float cam_position_x;
	float cam_position_y;
	float cam_rotation;
} robotData;

									//Description                 Values            Units     	  Represented values    Bits
typedef struct receivedData{
   uint8_t id:5;					//Robot ID                    [0, 15]           -             [0, 15]                  4
   uint16_t rho:11;					//Velocity length             [0, 2047]         0.004m/s      [0, 8.188]              11
   int16_t theta:11;				//Velocity angle              [-1024, 1023]     0.00307rad    [-pi, pi>               11
   uint8_t driving_reference:1;		//Driving reference           [0, 1]            -             {true, false}            1
   uint8_t use_cam_info:1;			//Use camera information      [0, 1]            -             {true, false}            1
   uint8_t use_angle:1;				//Use control using set angles[0, 1]			-			  {true, false}			   1
   int16_t angular_control:10;		//Reference angle or ang. vel [-512, 511]       0.098rad/s    [-8*2pi, 8*2pi]         10
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
}receivedData;

/*
 * For the Robot feedback we use the following packet definition

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
		v           Angular rate                [0, 4294967295]    [0, 32 bit float]       m/s/s         32    raw angular velocity from xsens

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
typedef struct feedbackData{
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

} feedbackData;

float uint32tofloat(uint32_t raw);
void printReceivedData(receivedData *input, uint8_t dataArray[RECEIVEPKTLEN]);
void printFeedbackData(feedbackData *input, uint8_t dataArray[32], uint8_t ackDataLength);

void packetToReceivedData(uint8_t input[RECEIVEPKTLEN], receivedData *output);
void convertReceivedData(receivedData *input, robotData *output);
void feedbackDataToPacket(feedbackData *input, uint8_t output[FULLFBPKTLEN]);

#endif /* PACKING_H_ */
