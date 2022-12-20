#include "stateEstimation.h"
#define RoT_BUFFER_SIZE 5

///////////////////////////////////////////////////// VARIABLES

static float state[4] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

/**
 * Translates the velocities from the wheels to the body velocities.
 * 
 * @param wheelSpeeds The speed achieved by each wheel.
 * @param output 	  The x, y and w speeds from a body perspective.
 */
static void wheels2Body(float wheelSpeeds[4], float output[3]);

/**
 * Smoothens out the IMU rate of turn data.
 * While this does decrease the response time slightly, it allows for smoother rotations.
 * 
 * @param rateOfTurn The current rate of turn as measured by the IMU.
 * @return float     The smoothed out rate of turn.
 */
float smoothen_rateOfTurn(float rateOfTurn);


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
	// TODO: when angular velocity control is implemented, you could improve state estimation
    //  by extending the Kalman filter to include angular velocity. You can combine the Xsens
    //  rateOfTurn with the angular velocity from wheel encoder data (already computed by wheels2Body)
    //  in this Kalman filter to improve the estimate.

	float vel[3] = {0.0f};
	wheels2Body(input->wheelSpeeds, vel);

	kalman_CalculateK();
	kalman_Update(input->xsensAcc, vel);

	float kalman_State[4] = {0.0f};
	kalman_GetState(kalman_State);

	// TODO check if input->visionYaw is scaled properly with the new REM messages
	yaw_Calibrate(input->xsensYaw, input->visionYaw, input->visionAvailable, input->rateOfTurn);
	float calibratedYaw = yaw_GetCalibratedYaw();

	state[body_x] = kalman_State[0];
	state[body_y] = kalman_State[2];
	state[body_w] = smoothen_rateOfTurn(input->rateOfTurn);
	state[body_yaw] = calibratedYaw;
}

float* stateEstimation_GetState() {
	return state;
}

float stateEstimation_GetFilteredRoT() {
    return state[body_w];
}


///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void wheels2Body(float wheelSpeeds[4], float output[3]){
	static const float denominatorA = rad_wheel / (2 * (pow(cosFront, 2) + pow(cosBack, 2)));
	static const float denominatorB = rad_wheel / (2 * (sinFront + sinBack));

	// Transformation from wheel speeds to translational and angular velocities (assuming no slip)
	output[body_x] = (cosFront * wheelSpeeds[wheels_RF] + cosBack * wheelSpeeds[wheels_RB] - cosBack * wheelSpeeds[wheels_LB] - cosFront * wheelSpeeds[wheels_LF]) * denominatorA;
	output[body_y] = (wheelSpeeds[wheels_RF] - wheelSpeeds[wheels_RB] - wheelSpeeds[wheels_LB] + wheelSpeeds[wheels_LF]) * denominatorB;
	output[body_w] = (sinBack * wheelSpeeds[wheels_RF] + sinFront * wheelSpeeds[wheels_RB] + sinFront * wheelSpeeds[wheels_LB] + sinBack * wheelSpeeds[wheels_LF]) * denominatorB / rad_robot;
}


float smoothen_rateOfTurn(float rateOfTurn){
    static float buffer[RoT_BUFFER_SIZE] = {0.0f}; // circular buffer
    static int idx = 0; // holds current index of buffer

    buffer[idx] = rateOfTurn;
    //idx = idx >= RoT_BUFFER_SIZE-1 ? 0 : idx + 1;
	idx = (idx+1) % RoT_BUFFER_SIZE;

    float avg = 0.0f;  // average of buffer, which is the smoothed rate of turn
    for (int i=0; i<RoT_BUFFER_SIZE; i++){
        avg += buffer[i];
    }
    return avg / RoT_BUFFER_SIZE;
} 