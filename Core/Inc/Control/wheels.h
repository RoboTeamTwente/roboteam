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



///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int wheelsInit();

int wheelsDeInit();

void setWheelSpeed(float wheelref[4]);

void setWheelRef(float input[4]);

float getWheelSpeed(wheel_names wheel);

int getPWM(wheel_names wheel);

#endif /* WHEELS_H_ */
