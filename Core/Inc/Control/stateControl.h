
/* Description: State control
 *
 * Instructions:
 * 1) Feedforwards the expected desired wheel stateControl based on the reference stateControl
 * 2) Adds a PID feedback to that signal based on the actual stateControl
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef DO_stateControl_H_
#define DO_stateControl_H_

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int stateControl_Init();

int stateControl_DeInit();

void stateControl_Update();

void stateControl_SetRef(float input[3]);

float* stateControl_GetWheelRef();

void stateControl_SetState(float input[3]);

void stateControl_ResetAngleI();

#endif /* DO_STATE_CONTROL_H_ */
