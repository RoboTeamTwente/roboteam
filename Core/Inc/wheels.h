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
#include "control_util.h"



///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum {
	wheels_uninitialized,
	wheels_ready
}wheels_states;// keeps track of the state of this system


///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

void init();

void deinit();

void wheelsCallback(float wheelref[4]);

void getEncoderData(int *encoderData);

#endif /* WHEELS_H_ */
