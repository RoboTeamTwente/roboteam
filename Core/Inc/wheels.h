
/* Description: wheel velocity controls
 *
 * Instructions:
 * 1) Feedforwards the PWM of the motors based on the desired wheel velocity
 * 2) Changes that signal using a PID and the actual wheel velocity
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef WHEELS_H_
#define WHEELS_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int wheels_Init();

int wheels_DeInit();

void wheels_Update();

void wheels_SetRef(float input[4]);

float* wheels_GetState();
//float wheels_GetState(wheel_names wheel);

void wheels_GetPWM(int wheelPWM[4]);
//int wheels_GetPWM(wheel_names wheel);

#endif /* WHEELS_H_ */
