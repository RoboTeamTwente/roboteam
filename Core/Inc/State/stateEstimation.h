
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

#include "../Inc/State/yawCalibration.h"
#include "../Inc/State/kalman.h"
#include "../Util/control_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int state_Init();

int state_Deinit();

void state_Update(float xsensData[3], float wheelSpeeds[4], float visionYaw, bool visionAvailable);

void state_GetState(float output[4]);

#endif /* DO_STATEESTIMATION_H_ */
