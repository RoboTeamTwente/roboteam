/*
 * vel_control.h
 *
 *  Created on: Jan 23, 2019
 *      Author: simen
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

#ifndef DO_VEL_CONTROL_H_
#define DO_VEL_CONTROL_H_

#include "../Util/control_util.h"
#include "stdbool.h"

///////////////////////////////////////////////////// DEFINITIONS



///////////////////////////////////////////////////// STRUCTS



///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int vel_control_Init();

int vel_control_DeInit();

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3], bool use_global_ref);

#endif /* DO_VEL_CONTROL_H_ */
