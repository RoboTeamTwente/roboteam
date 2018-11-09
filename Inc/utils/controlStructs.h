/*
 * controlStructs.h
 *
 *  Created on: Nov 9, 2018
 *      Author: kjhertenberg
 */

/*
Description: declares the structs used by the control files

Instructions:

Extra functions:

GPIO Pins: None

Notes:
Still need to add the right specs
*/

#ifndef UTILS_CONTROLSTRUCTS_H_
#define UTILS_CONTROLSTRUCTS_H_

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF,
}wheel_names;

typedef struct PIDconstants {
	float kP;
	float kI;
	float kD;
}PIDconstants;


#endif /* UTILS_CONTROLSTRUCTS_H_ */
