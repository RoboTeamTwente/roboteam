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

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum{
	geneva_idle,		// in idle it will do nothing
	geneva_setup,		// at startup it will try to find the edge sensor
	geneva_returning,	// when moving back to the initial/zero position
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

 /*	should be called in the while loop to
  * 	update state and such
  */
void geneva_Update();

/*	this function calls its pid controller and should be called with a specific time
 *
 */
void geneva_Control();

/*	for debugging, sets the current geneva state
 *
 */
void geneva_SetState(geneva_states state);

/*	returns the state from geneva_states
 *
 */
geneva_states geneva_GetState();

/*	Set the position to one of the values of geneva_positions
 *
 */
geneva_positions geneva_SetPosition(geneva_positions position);

/*	returns the current positions
 * 	from -2 to 2
 */
geneva_positions geneva_GetPosition();

/*	returns the raw encoder value
 *
 */
int geneva_Encodervalue();

#endif /* GENEVA_GENEVA_H_ */
