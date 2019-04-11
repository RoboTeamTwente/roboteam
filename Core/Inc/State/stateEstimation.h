/*
 * stateEstimation.h
 *
 *  Created on: Jan 25, 2019
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

#ifndef DO_STATEESTIMATION_H_
#define DO_STATEESTIMATION_H_
#include "stdbool.h"

///////////////////////////////////////////////////// DEFINITIONS



///////////////////////////////////////////////////// STRUCTS



///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int stateInit();
int stateDeInit();
void estimateState(float State[3], float xsensData[3], bool visionAvailable);
void getState(float output[4]);

#endif /* DO_STATEESTIMATION_H_ */
