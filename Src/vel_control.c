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
#define rad_robot 0.0775F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.0275F 	// wheel radius (m)
#define cos60 0.5F		// cosine of 60 degrees (wheel angle is at 60 degrees)
#define sin60 0.866F	// sine of 60 degrees

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//transfer body velocity to necessary wheel speed
static void body2Wheels(float input[2], float output[4]);

//transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float  yaw);

//scales and limit the signal
static void scaleAndLimit(float wheel_ref[4]);

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int vel_control_Init(){
	HAL_TIM_Base_Start_IT(&htim7);
	//TODO: add control values based on tests
	angleK.kP = 0;//kp
	angleK.kI = 0;//ki
	angleK.kD = 0;//kd
	angleK.prev_e = 0;//always starts as zero
	angleK.timeDiff = TIME_DIFF;//
	wheelK.kP = 0;//kp
	wheelK.kI = 0;//ki
	wheelK.kD = 0;//kd
	wheelK.prev_e = 0;//always starts as zero
	angleK.timeDiff = TIME_DIFF;
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

	float angleErr = constrainAngle((vel_ref[body_w] - xsensData[body_w]));//constrain it to one circle turn
	float angleComp = PID(angleErr, angleK);// PID control from control_util.h
	float velLocalRef[2] = {0};
	global2Local(vel_ref, velLocalRef, xsensData[body_w]); //transfer global to local

	// PID control from control_util.h
	velLocalRef[body_x] += PID((velLocalRef[body_x]-xsensData[body_x]), angleK);; //error compensation plus requested velocity
	velLocalRef[body_y] += PID((velLocalRef[body_y]-xsensData[body_y]), angleK);;

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

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
	output[wheels_RF] = -(1/sin60*input[body_x] + 1/cos60*input[body_y])*rad_wheel/4;
	output[wheels_RB] = -(1/sin60*input[body_x] - 1/cos60*input[body_y])*rad_wheel/4;
	output[wheels_LB] = -(-1/sin60*input[body_x] - 1/cos60*input[body_y])*rad_wheel/4;
	output[wheels_LF] = -(-1/sin60*input[body_x] + 1/cos60*input[body_y])*rad_wheel/4;
}

static void global2Local(float input[2], float output[2], float  yaw){
	//trigonometry
	output[body_x] = cos(yaw)*input[body_x]-sin(yaw)*input[body_y];
	output[body_y] = sin(yaw)*input[body_x]+cos(yaw)*input[body_y];
}

static void scaleAndLimit(float wheel_ref[4]){
	//TODO: add some limitation stuff here
}

//Scales the angle to the range Pi to -Pi in radians
static float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0)
        x += 2*M_PI;
    return x - M_PI;
}
