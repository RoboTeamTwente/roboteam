
#include "../Inc/State/stateEstimation.h"

///////////////////////////////////////////////////// VARIABLES

static float state[3] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Transforms wheel speed to body velocity
static void wheels2Body(float wheelSpeeds[4], float output[3]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int state_Init(){
	kalman_Init();
	return 0;
}

int state_Deinit(){
	kalman_Deinit();
	return 0;
}

void state_Update(float xsensData[3], float wheelSpeeds[4], float visionYaw, bool visionAvailable) {

	float acc[2] = {0.0f};
	acc[body_x] = xsensData[body_x];
	acc[body_y] = xsensData[body_y];

	float vel[2]= {0.0f};
	wheels2Body(wheelSpeeds, vel);

	kalman_CalculateK();
	kalman_Update(acc, vel);

	float kalman_State[4] = {0.0f};
	kalman_GetState(kalman_State);

	yaw_Calibrate(xsensData[body_w], visionYaw, visionAvailable);
	float calibratedYaw = yaw_GetCalibratedYaw();

	state[body_x] = kalman_State[0];
	state[body_y] = kalman_State[2];
	state[body_w] = calibratedYaw;
}

void state_GetState(float output[4]){
	for (int i=0; i<4; i++){
		output[i] = state[i];
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void wheels2Body(float wheelSpeeds[4], float output[3]){
	//Applying transpose(M_inv) matrix to go from wheel angular velocity to body velocity (assuming no slip)
	output[body_x] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] - wheelSpeeds[wheels_LF])/sin60 * rad_wheel/4;
	output[body_y] = (wheelSpeeds[wheels_RF] - wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/cos60 * rad_wheel/4;
	output[body_w] = (wheelSpeeds[wheels_RF] + wheelSpeeds[wheels_RB] + wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF])/rad_robot * rad_wheel/4;
}
