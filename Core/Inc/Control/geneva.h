/*
 * geneva.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
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

#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_


///////////////////////////////////////////////////// STRUCTS

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

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void geneva_Init();
void geneva_Deinit();
void geneva_Update();
geneva_positions geneva_SetPosition(geneva_positions position);
geneva_positions geneva_GetPosition();

#endif /* GENEVA_GENEVA_H_ */
