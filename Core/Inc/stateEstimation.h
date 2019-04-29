
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

int stateEstimation_Init();

int stateEstimation_DeInit();

void stateEstimation_Update(StateInfo* input);

float* stateEstimation_GetState();

#endif /* DO_STATEESTIMATION_H_ */
