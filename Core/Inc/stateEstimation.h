
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

///////////////////////////////////////////////////// STRUCTS

typedef struct StateInfo {
	float visionYaw;
	bool visionAvailable;
	float* xsensAcc;
	float xsensYaw;
	float* wheelSpeeds;
} StateInfo;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int state_Init();

int state_Deinit();

void state_Update(StateInfo* input);

float* state_GetState();
//void state_GetState(float output[3]);

#endif /* DO_STATEESTIMATION_H_ */
