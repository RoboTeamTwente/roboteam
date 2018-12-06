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


#include <stdbool.h>
#include "Utils/control_util.h"



///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum {
	wheels_uninitialized,
	wheels_ready
}wheels_states;// keeps track of the state of this system

int wheels_state = wheels_uninitialized;

PIDvariables wheelsK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = 0.1
};

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC



#endif /* WHEELS_H_ */
