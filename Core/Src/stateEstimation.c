
#include "../Inc/stateEstimation.h"

///////////////////////////////////////////////////// VARIABLES

static float state[3] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Transforms wheel speed to body velocity
static void wheels2Body(float wheelSpeeds[4], float output[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int stateEstimation_Init(){
	kalman_Init();
	return 0;
}

int stateEstimation_DeInit(){
	kalman_DeInit();
	return 0;
}

void stateEstimation_Update(StateInfo* input) {
	float vel[2] = {0.0f};
	wheels2Body(input->wheelSpeeds, vel);

	kalman_CalculateK();
	kalman_Update(input->xsensAcc, vel);

	float kalman_State[4] = {0.0f};
	kalman_GetState(kalman_State);

	yaw_Calibrate(input->xsensYaw, input->visionYaw, input->visionAvailable, input->rateOfTurn);
	float calibratedYaw = yaw_GetCalibratedYaw();

	state[body_x] = kalman_State[0];
	state[body_y] = kalman_State[2];
	state[body_w] = calibratedYaw;
}

float* stateEstimation_GetState() {
	return state;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void wheels2Body(float wheelSpeeds[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] - wheelSpeeds[wheels_LF])/sin60 * rad_wheel/4;
	output[body_y] = (wheelSpeeds[wheels_RF] - wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/cos60 * rad_wheel/4;
	output[body_w] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] + wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/rad_robot * rad_wheel/4;
}
