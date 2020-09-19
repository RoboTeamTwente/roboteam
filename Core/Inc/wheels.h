
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

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int wheels_Init();

int wheels_DeInit();

void wheels_Update();

void wheels_SetRef(float input[4]);

float* wheels_GetState();

int* wheels_GetPWM();

// Returns whether brakes are activated
bool wheels_IsBraking();

// Activate or deactivate braking
void wheels_Brake(bool brake);

#endif /* WHEELS_H_ */
