
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

#include "yawCalibration.h"
#include "kalman.h"
#include "control_util.h"

///////////////////////////////////////////////////// STRUCTS

typedef struct StateInfo {
	float visionYaw;					// The yaw for this robot as indicated by vision
	bool visionAvailable;				// Wether vision data can be used at this point
	float xsensAcc[2];					// The acceleration as measured by the IMU in the X and Y directions
	float xsensYaw;						// They yaw for this robot as indicated by the IMU
	float rateOfTurn;					// [rad/s]
	float wheelSpeeds[4];				// The speed for each wheel 
	float dribblerSpeed;				// The measured speed of the dribbler
	float dribblerFilteredSpeed;		// The filtered speed of the dribbler
	float dribbleSpeedBeforeGotBall;	// The speed of the dribbler before it had a ball
} StateInfo;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

/**
 * Initializes the state estimation, which for now just sets up the kalman filter.
 */
int stateEstimation_Init();

/**
 * De-initializes the kalman filter.
 */
int stateEstimation_DeInit();

/**
 * Updates the current state based on the measured wheel speeds, vision yaw and acceleration
 * 
 * @param input The vision, IMU and encoder data used for state estimation
 */
void stateEstimation_Update(StateInfo* input);

/**
 * Get the current estimated state
 * 
 * @return float* The curent state for x, y, w and yaw
 */
float* stateEstimation_GetState();

/**
 * Identical to stateEstimation_GetState(), but only returns w.
 * 
 * @return float Rate of return
 */
float stateEstimation_GetFilteredRoT();

void stateControl_ResetPID();

#endif /* DO_STATEESTIMATION_H_ */
