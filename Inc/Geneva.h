/*
 * Geneva.h
 *
 *  Created on: Nov 9, 2018
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

#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_
#include <stdbool.h>

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum{
	geneva_idle,		// We are not doing anything
	geneva_setup,		// at startup it will try to find the edge sensor
	geneva_running		// when being operational
}geneva_states;

typedef enum{
	geneva_leftleft,
	geneva_left,
	geneva_middle,
	geneva_right,
	geneva_rightright,
	geneva_none			// While rotating
}geneva_positions;

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

void geneva_Init();

void geneva_Deinit();

void geneva_Callback(int genevaStateRef);

#endif /* GENEVA_GENEVA_H_ */
