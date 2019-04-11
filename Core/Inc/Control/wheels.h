/*
 * wheels.h
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

/* Description:
 *
 * Instructions:
 * 1)
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef WHEELS_H_
#define WHEELS_H_
#include <stdbool.h>
#include "../Util/control_util.h"

///////////////////////////////////////////////////// STRUCTS

typedef enum {
	wheels_uninitialized,
	wheels_ready
}wheels_states;// keeps track of the state of this system

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int wheelsInit();

int wheelsDeInit();

void setWheelSpeed(float wheelref[4]);

float getWheelSpeed(wheel_names wheel);

int getPWM(wheel_names wheel);

#endif /* WHEELS_H_ */
