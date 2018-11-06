/*
 * control.c
 *
 *  Created on: Nov 6, 2018
 *      Author: kjhertenberg
 */

#include <stdio.h>
#include <math.h>
#include <vel_control.h>
#include "tim.h"

///////////////////////////////////////////////////// DEFINITIONS
// Basically set constants
#define CALIBRATE_AFTER 6500 // time after which yaw calibration will start (ms)
#define TIME_DIFF 0.01F // time difference due to 100Hz
#define R 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define r 0.0275F 	// wheel radius (m)
#define c 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define s 0.866F	// sine of 60 degrees
#define PWM_CUTOFF 3.0F // below this value the motor PWM is set to 0 (see wheels.c)

uint start_time;

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void body2Wheels(float input[2], float output[4]);

static void global2Local(float input[3], float output[2], float  yaw);

static float singlePID(float ref, float state, float K[3]);

static void doublePID(float ref[2], float state[3], float K[3], float Adjuster[2]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	start_time = HAL_GetTick();
	return 0;
}

void vel_control_Callback(float wheel_ref[4], float Xsensdata[3], float vel_ref[3]){
	float angleK[3] = {0,0,0};//kp,ki,kp
	float wheelK[3] = {0,0,0};//kp,ki,kp
	float velLocalRef[2] = {0};


	float angleComp = singlePID(vel_ref[2], Xsensdata[2], angleK);// angle control

	global2Local(vel_ref, velLocalRef, Xsensdata[2]); //transfer global to local

	float Adjuster[2] = {0};
	doublePID(velLocalRef, Xsensdata, wheelK, Adjuster);
	Adjuster[0] += velLocalRef[0];
	Adjuster[1]	+= velLocalRef[1];

	body2Wheels(Adjuster, wheel_ref);

	for (int i = 0; i <= 4; ++i){
		wheel_ref[i] += angleComp;
	}
	//TODO:
	//combine xsens with encoder?
	//what in the case if a wheel is slipping?
	//individual wheel check?

	return;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float input[2], float output[4]){
	output[wheels_RF] = -(1/s*input[body_x] + 1/c*input[body_y])*r/4;
	output[wheels_RB] = -(1/s*input[body_x] - 1/c*input[body_y])*r/4;
	output[wheels_LB] = -(-1/s*input[body_x] - 1/c*input[body_y])*r/4;
	output[wheels_LF] = -(-1/s*input[body_x] + 1/c*input[body_y])*r/4;
}

static void global2Local(float input[3], float output[2], float  yaw){
	output[body_x] = cos(yaw)*input[0]-sin(yaw)*input[1];
	output[body_y] = sin(yaw)*input[0]+cos(yaw)*input[1];
}

static float singlePID(float ref, float state, float K[3]){
	static float prev_e = 0;
	static float I = 0;
	float err = ref - state;
	float P = K[0]*err;
	I += K[1]*err*TIME_DIFF;
	float D = (K[2]*(err-prev_e))/TIME_DIFF;
	prev_e = err;
	float Adjuster = P + I + D;
	return Adjuster;
}

static void doublePID(float ref[2], float state[3], float K[3], float Adjuster[2]){
		static float prev_e[2] = {0};
		float P[2] = {0};
		static float I[2] = {0};
		float D[2] = {0};
		float err[2] = {0};
		for (int i = 0; i<2; ++i){
			err[i] = ref[i] - state[i];
			P[i] = K[0]*err[i];
			I[i] += K[1]*err[i]*TIME_DIFF;
			D[i] = (K[2]*(err[i]-prev_e[i]))/TIME_DIFF;
			prev_e[i] = err[i];
			Adjuster[i] = P[i] + I[i] + D[i];
		}
}
