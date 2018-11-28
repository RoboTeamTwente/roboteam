/*
 * control.h
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */
/*
Description: Determines the wanted wheel speed, based on received data

Instructions:
1) Initialize
2) Receives data of it's state
3) Combine data, not in here yet
4) Transform data to right frame and units
5) Apply control functions
6) Scale and limit the outgoing signal

Extra functions:

GPIO Pins: None

Notes:
Still need to add the right specs
*/

#ifndef CONTROL_H_
#define CONTROL_H_
#define TIME_DIFF 0.01F // time difference due to 100Hz
#include <stdbool.h>
#include "Utils/control_util.h"

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

	//TODO: add control values based on tests
PIDvariables angleK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velxK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables velyK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};



///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

int vel_control_Init();

void vel_control_Callback(float wheel_ref[4], float Xsensdata[3], float vel_ref[3]);

#endif /* DO_DO_H_ */

