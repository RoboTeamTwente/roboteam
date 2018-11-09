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
#define TIME_DIFF 0.01F // time difference due to 100Hz
#define R 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define r 0.0275F 	// wheel radius (m)
#define c 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define s 0.866F	// sine of 60 degrees

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

//PID for the angle specifically, (since it's rotation the coordinates are non-euclidean)
static float anglePID(float ref, float state, struct PIDconstants K);

//PID for 2 index arrays
static void doublePID(float PIDvalue[2], float ref[2], float state[3], struct PIDconstants K);

//function to prevent asking for impossible stuff
static void scaleAndLimit(float wheel_ref[4]);

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	angleK.kP = 0;//kp
	angleK.kI = 0;//ki
	angleK.kD = 0;//kd
	wheelK.kP = 0;//kp
	wheelK.kI = 0;//ki
	wheelK.kD = 0;//kd
	return 0;
}

void vel_control_Callback(float wheel_ref[4], float xsensData[3], float vel_ref[3]){

	/*For dry testing
	xsensData[body_w] = MT_GetAngles()[2]/180*M_PI;
	vel_ref[body_x] = 0;
	vel_ref[body_y] = 0;
	vel_ref[body_w] = 0*M_PI;
	xsensData[body_x] = 0;
	xsensData[body_y] = 0;
	xsensData[body_w] = 0*M_PI;
	*/

	float angleComp = anglePID(vel_ref[body_w], xsensData[body_w], angleK);// angle control
	float velLocalRef[2] = {0};
	global2Local(vel_ref, velLocalRef, xsensData[body_w]); //transfer global to local

	float PIDoutput[2] = {0};
	doublePID(PIDoutput, velLocalRef, xsensData, wheelK); //xy velocity control
	PIDoutput[body_x] += velLocalRef[body_x]; //error compensation plus requested velocity
	PIDoutput[body_y] += velLocalRef[body_y];

	body2Wheels(wheel_ref, PIDoutput); //translate velocity to wheel speed

	for (int i = 0; i < 4; ++i){
		wheel_ref[i] += angleComp; //add necessary rotation value
	}

	scaleAndLimit(wheel_ref);

	//TODO:
	//combine xsens with encoder?
	//what in the case if a wheel is slipping?
	//individual wheel check?

	return;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float output[4], float input[2]){
	//mixing matrix
	output[wheels_RF] = -(1/s*input[body_x] + 1/c*input[body_y])*r/4;
	output[wheels_RB] = -(1/s*input[body_x] - 1/c*input[body_y])*r/4;
	output[wheels_LB] = -(-1/s*input[body_x] - 1/c*input[body_y])*r/4;
	output[wheels_LF] = -(-1/s*input[body_x] + 1/c*input[body_y])*r/4;
}

static void global2Local(float input[2], float output[2], float  yaw){
	//trigonometry
	float globalXVel = input[body_x];//cos(input[1])*input[0];
	float globalYVel = input[body_y];//sin(input[1])*input[0];
	output[body_x] = cos(yaw)*globalXVel-sin(yaw)*globalYVel;
	output[body_y] = sin(yaw)*globalXVel+cos(yaw)*globalYVel;
}

static float anglePID(float ref, float state, struct PIDconstants K){
	static float prev_e = 0;
	static float I = 0;
	float err = constrainAngle(ref - state); //constrain it to one circle turn
	float P = K.kP*err;
	I += K.kI*err*TIME_DIFF;
	float D = (K.kD*(err-prev_e))/TIME_DIFF;
	prev_e = err;
	float PIDvalue = P + I + D;
	return PIDvalue;
}

// PID for a 2 index array
static void doublePID(float PIDvalue[2], float ref[2], float state[3], struct PIDconstants K){
		static float prev_e[2] = {0};
		float P[2] = {0};
		static float I[2] = {0};
		float D[2] = {0};
		float err[2] = {0};
		for (int i = 0; i<2; ++i){
			err[i] = ref[i] - state[i];
			P[i] = K.kP*err[i];
			I[i] += K.kI*err[i]*TIME_DIFF;
			D[i] = (K.kD*(err[i]-prev_e[i]))/TIME_DIFF;
			prev_e[i] = err[i];
			PIDvalue[i] = P[i] + I[i] + D[i];
		}
}

static void scaleAndLimit(float wheel_ref[4]){
	//add some limitation stuff here
}

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
