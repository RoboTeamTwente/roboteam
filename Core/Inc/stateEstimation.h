
/* Description: Estimates the state
 *
 * Instructions:
 * 1) Takes all the data received/gathered by the robot
 * 2) Uses the kalman filter and yawcalibration to filter the data
 * 3) creates an up to date state estimation
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef DO_STATEESTIMATION_H_
#define DO_STATEESTIMATION_H_

#include "../Lib/yawCalibration.h"
#include "../Lib/kalman.h"
#include "../Util/control_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int state_Init();

int state_Deinit();

void state_Update(float xsensData[3], float wheelSpeeds[4], float visionYaw, bool visionAvailable);

void state_GetState(float output[4]);

#endif /* DO_STATEESTIMATION_H_ */
