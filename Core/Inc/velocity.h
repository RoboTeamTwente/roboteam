
/* Description: Velocity control
 *
 * Instructions:
 * 1) Feedforwards the expected desired wheel velocity based on the reference velocity
 * 2) Adds a PID feedback to that signal based on the actual velocity
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef DO_VELOCITY_H_
#define DO_VELOCITY_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int velocity_Init();

int velocity_DeInit();

void velocity_Update();

void velocity_SetRef(float input[3]);

void velocity_GetWheelRef(float output[4]);

void velocity_SetState(float input[3]);

#endif /* DO_VEL_CONTROL_H_ */
