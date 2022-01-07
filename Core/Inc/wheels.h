
/* Description: wheel velocity controls
 *
 * Instructions:
 * 1) Initialize the wheels by calling wheels_Init()
 * 2) Unbrake the wheels by calling wheels_Unbrake()
 * 3) Set the wheel velocities by calling wheels_SetSpeeds()
 * 4) Repeatedly call wheels_Update() every 10ms (as dictated by the variable TIME_DIFF) to ensure the wheels reach their speeds
 * ==== instructions below not needed per se. Turning off the robot will do fine ====
 * 5) Stop the robot by calling wheels_Stop()
 * 6) Turn on the brakes by calling wheels_Brake()
 * 7) Deinitialize the wheels by calling wheels_Deinitialize()
 * 
 * Extra functions:
 * 1) wheels_GetMeasuredSpeeds to get the last measured wheel speeds
 * 2) wheels_GetPWM to get the current wheel PWM values
 * 3) wheels_GetWheelsBraking to get the current status of the brakes
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

// Initializes the PIDs / PWM timers / encoders
void wheels_Init();
// Deinitializes the PIDs / PWM timers / encoders
void wheels_DeInit();
// Stops the wheels without deinitializing them 
void wheels_Stop();
// Updates the wheels towards the commanded wheel speeds using the encoders and a PID controller.
void wheels_Update();
// Stores the commanded wheel speeds, in rad/s, to be used in the next wheels_Update() call
void wheels_SetSpeeds(const float speeds[4]);
// Get the last measured wheel speeds in rad/s
void wheels_GetMeasuredSpeeds(float speeds[4]);
// Get the current wheel PWMs
void wheels_GetPWM(uint32_t pwms[4]);
// Get the current status of the brakes
bool wheels_GetWheelsBraking();
// Enable the brakes
void wheels_Brake();
// Disable the brakes
void wheels_Unbrake();

#endif /* WHEELS_H_ */
