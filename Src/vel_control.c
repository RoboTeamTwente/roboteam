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
	return 0;
}

void vel_control_Callback(float wheel_ref[4], float State[3], float vel_ref[3], float yawref){

	/*For dry testing
	State[body_w] = MT_GetAngles()[2]/180*M_PI;
	vel_ref[body_x] = 0;
	vel_ref[body_y] = 0;
	vel_ref[body_w] = 0*M_PI;
	State[body_x] = 0;
	State[body_y] = 0;
	State[body_w] = 0*M_PI;
	*/

	float angleErr = constrainAngle((yawref - State[body_w]));//constrain it to one circle turn
	float angleComp = PID(angleErr, &angleK);// PID control from control_util.h
	float velLocalRef[3] = {0};
	global2Local(vel_ref, velLocalRef, State[body_w]); //transfer global to local

	// PID control from control_util.h
	velLocalRef[body_x] += PID((velLocalRef[body_x]-State[body_x]), &velxK); //error compensation plus requested velocity
	velLocalRef[body_y] += PID((velLocalRef[body_y]-State[body_y]), &velyK);
	velLocalRef[body_w] += PID((velLocalRef[body_w]-State[body_w]), &velwK);

	body2Wheels(wheel_ref, velLocalRef); //translate velocity to wheel speed

	for (int i = 0; i < 4; ++i){
		wheel_ref[i] += angleComp; //add necessary rotation value
	}

	scaleAndLimit(wheel_ref);

	return;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelspeed[4], float velocity[3]){
	//mixing matrix
	//TODO check minuses
	float velx2wheel = sin60*velocity[body_x]/rad_wheel;
	float vely2wheel = cos60*velocity[body_y]/rad_wheel;
	float rot2wheel = rad_robot*velocity[body_w]/rad_wheel;
	wheelspeed[wheels_RF] = -(velx2wheel + vely2wheel + rot2wheel);
	wheelspeed[wheels_RB] = -(velx2wheel - vely2wheel + rot2wheel);
	wheelspeed[wheels_LB] = -(-velx2wheel - vely2wheel + rot2wheel);
	wheelspeed[wheels_LF] = -(-velx2wheel + vely2wheel + rot2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cos(yaw)*global[body_x]-sin(yaw)*global[body_y];
	local[body_y] = sin(yaw)*global[body_x]+cos(yaw)*global[body_y];
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
