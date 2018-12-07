/*
 * wheels.h
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

/*Description:

Instructions:
1)

Extra functions:

Notes:


*/


#ifndef WHEELS_H_
#define WHEELS_H_
#define TIME_DIFF 0.01F // time difference due to 100Hz
#include <stdbool.h>
#include "Utils/control_util.h"



///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum {
	wheels_uninitialized,
	wheels_ready
}wheels_states;// keeps track of the state of this system

int wheels_state = wheels_uninitialized;

//TODO: add control values based on tests
PIDvariables RFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};PIDvariables RBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables LBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables LFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};


///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

void init();

void deinit();

void wheelsCallback(float wheelref[4]);

#endif /* WHEELS_H_ */
