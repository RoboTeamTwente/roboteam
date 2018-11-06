/*
 * control.h
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */
/*
Description: ...

Instructions:
1) ...

Extra functions:

GPIO Pins: None

Notes:
*/

#ifndef CONTROL_H_
#define CONTROL_H_
#include <stdbool.h>

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs
typedef enum {
	DO_succes,
	DO_error
}DO_States;

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


///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

int vel_control_Init();
void vel_control_Callback(float wheel_ref[4], float Xsensdata[3], float vel_ref[3]);

#endif /* DO_DO_H_ */

