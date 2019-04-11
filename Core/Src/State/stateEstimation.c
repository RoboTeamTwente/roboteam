/*
 * stateEstimation.c
 *
 *  Created on: Jan 25, 2019
 *      Author: simen
 */

#include "stateEstimation.h"
#include "Kalman.h"
#include "../Util/control_util.h"

///////////////////////////////////////////////////// VARIABLES

static float state[3] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void wheels2Body(float wheelSpeeds[4], float output[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int stateInit(){
	kalmanInit();
	yawCalibrationInit();
	return 0;
}

int stateDeInit(){
	kalmanDeinit();
	yawCalibrationDeinit();
	return 0;
}

void estimateState(float xsensData[3], float wheelSpeeds[4], float visionYaw, bool visionAvailable) {

	float acc[2] = {0.0f};
	acc[body_x] = xsensData[body_x];
	acc[body_y] = xsensData[body_y];

	float vel[2]= {0.0f};
	wheels2Body(wheelSpeeds, vel);

	KalmanK();
	KalmanState(acc, vel);
	float Kstate[4] = {0.0f};
	getKalmanState(Kstate);
	state[body_x] = Kstate[0];
	state[body_y] = Kstate[2];

	calibrateXsens(xsensData, visionYaw, visionAvailable);
	state[body_w] = xsensData[body_w];
}

void getState(float output[4]){
	for (int i=0; i<4; i++){
		output[i] = state[i];
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

//multiplies a 3*4 matrix by a vector of 4 elements.
static void wheels2Body(float wheelSpeeds[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] - wheelSpeeds[wheels_LF])/sin60 * rad_wheel/4;
	output[body_y] = (wheelSpeeds[wheels_RF] - wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/cos60 * rad_wheel/4;
	output[body_w] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] + wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/rad_robot * rad_wheel/4;
}
