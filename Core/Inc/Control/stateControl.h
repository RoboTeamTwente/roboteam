
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
#include "REM_RobotSetPIDGains.h"

#define default_P_gain_x 0.2
#define default_I_gain_x 0.0
#define default_D_gain_x 0.0

#define default_P_gain_y 0.3
#define default_I_gain_y 0.0
#define default_D_gain_y 0.0

#define default_P_gain_w 0.25
#define default_I_gain_w 5.0
#define default_D_gain_w 0.0

#define default_P_gain_yaw 20.0
#define default_I_gain_yaw 5.0
#define default_D_gain_yaw 0.0

#define default_P_gain_wheels 2.0
#define default_I_gain_wheels 0.0
#define default_D_gain_wheels 0.0

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int stateControl_Init();

int stateControl_DeInit();

void stateControl_Update();

void stateControl_SetRef(float input[4]);

float* stateControl_GetWheelRef();

void stateControl_SetState(float input[4]);

void stateControl_GetPIDGains(PIDvariables gains[4]);

float stateControl_GetIntegral(body_handles direction);

void stateControl_useAbsoluteAngle(bool angularControl);

void stateControl_SetPIDGains(REM_RobotSetPIDGains* pidConfig);

void stateControl_ResetAngleI();

void stateControl_ResetPID();

#endif /* DO_STATE_CONTROL_H_ */
