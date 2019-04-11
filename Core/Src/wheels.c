/*
 * wheels.c
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

#include "wheels.h"
#include <stdio.h>
#include <math.h>
#include "tim_util.h"
#include "gpio_util.h"


///////////////////////////////////////////////////// DEFINITIONS

#define MAX_PWM 2400
#define PWM_CUTOFF 3.0F			// arbitrary treshold below PWM_ROUNDUP
#define PWM_ROUNDUP 3.1F 		// below this value the motor driver is unreliable

#define gearratio 2.5f
#define max_voltage 12//see datasheet
#define sconstant 374//RPM/V see datasheet
#define wconstant (float)60/(2*M_PI*(float)sconstant) // RPM/V to V/w
#define PWM2Omega (float)(wconstant*MAX_PWM/max_voltage)*gearratio //(V/W)*(pwm/voltage)


///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void SetPWM(int pwm[4]);

static void SetDir(bool direction[4]);

static float deriveEncoder(int encoderData, int prev_encoderData);

static void limitScale();


int wheels_state = wheels_uninitialized;
//TODO: add control values based on tests
PIDvariables RFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};PIDvariables RBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables LBK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};
PIDvariables LFK = {
		.kP = 0,//kp
		.kI = 0,//ki
		.kD = 0,//kd
		.I = 0,//always starts as zero
		.prev_e = 0,//always starts as zero
		.timeDiff = TIME_DIFF
};

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void init(){
	wheels_state = wheels_ready;
	HAL_TIM_Base_Start(ENC_RF); //RF
	HAL_TIM_Base_Start(ENC_RB); //RB
	HAL_TIM_Base_Start(ENC_LB); //LB
	HAL_TIM_Base_Start(ENC_LF); //LF
	HAL_TIM_Base_Start(&htim5); //TIME
	startPWM(PWM_RF);
	startPWM(PWM_RB);
	startPWM(PWM_LB);
	startPWM(PWM_LF);
}

void deinit(){
	wheels_state = wheels_uninitialized;
	HAL_TIM_Base_Stop(ENC_RF); //RF
	HAL_TIM_Base_Stop(ENC_RB); //RB
	HAL_TIM_Base_Stop(ENC_LB); //LB
	HAL_TIM_Base_Stop(ENC_LF); //LF
	HAL_TIM_Base_Stop(&htim5); //TIME
	stopPWM(PWM_RF);
	stopPWM(PWM_RB);
	stopPWM(PWM_LB);
	stopPWM(PWM_LF);
}

void wheelsCallback(float wheelref[4]){

	//TODO: add braking for rapid switching direction
	//TODO: Add slipping case
	/* dry testing
	wheelref[0] = 0;
	wheelref[1] = 0;
	wheelref[2] = 0;
	wheelref[3] = 0;
	*/
	static bool direction[4] = {0}; // 0 is counter clock-wise TODO:confirm
	static int prev_state[4] = {0};
	int state[4] = {0};
	int output[4] = {0};
	int pwm[4] = {0};
	float err[4] = {0};
	float wheelspeed[4] = {0};
	switch(wheels_state){
		default:
			break;
		case wheels_ready:

			//get encoder data
			getEncoderData(state);

			//derive wheelspeed
			for(int i = wheels_RF; i <= wheels_LF; i++){
				wheelspeed[i] = deriveEncoder(state[i], prev_state[i]);
				err[i] = wheelref[i]-wheelspeed[i];
				prev_state[i] = state[i];
			}

			//combine reference and PID
			output[wheels_RF] = wheelref[wheels_RF] + PID(err[wheels_RF], &RFK);
			output[wheels_RB] = wheelref[wheels_RB] + PID(err[wheels_RB], &RBK);
			output[wheels_LB] = wheelref[wheels_LB] + PID(err[wheels_LB], &LBK);
			output[wheels_LF] = wheelref[wheels_LF] + PID(err[wheels_LF], &LFK);


			limitScale(output, pwm, direction);
			SetDir(direction);
			SetPWM(pwm);
			break;
		}
	return;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
static void limitScale(float output[4], float pwm[4], float direction[4]){

	//Scale
	for(int i = wheels_RF; i <= wheels_LF; i++){
		output[i] = PWM2Omega*output[i];
	}
	//Limit
	for(int i = wheels_RF; i <= wheels_LF; i++){
		if(output[i] <= -1.0F){
			pwm[i] = -output[i];
			direction[i] = 1;
		}else if(output[i] >= 1.0F){
			pwm[i] = output[i];
			direction[i] = 0;
		}
		else {
			pwm[i] = 0.0F;
		}
		if(pwm[i] < PWM_CUTOFF){
			pwm[i] = 0.0F;
		}else if(pwm[i] < PWM_ROUNDUP){
			pwm[i] = PWM_ROUNDUP;
		}else if(pwm[i] > MAX_PWM){
			pwm[i] = MAX_PWM;
		}
	}
}

void getEncoderData(int* encoderData){
	*encoderData++ = __HAL_TIM_GET_COUNTER(ENC_RF);
	*encoderData++ = __HAL_TIM_GET_COUNTER(ENC_RB);
	*encoderData++ = __HAL_TIM_GET_COUNTER(ENC_LB);
	*encoderData++ = __HAL_TIM_GET_COUNTER(ENC_LF);
}

static float deriveEncoder(int encoderData, int prev_encoderData){
	float wheel_speed = (encoderData-prev_encoderData)/TIME_DIFF;
	return wheel_speed;
}

static void SetPWM(int pwm[4]){
	setPWM(PWM_RF,pwm[0]);
	setPWM(PWM_RB,pwm[1]);
	setPWM(PWM_LB,pwm[2]);
	setPWM(PWM_LF,pwm[3]);
}

static void SetDir(bool direction[4]){
	set_pin(RF_DIR_pin,direction[0]);
	set_pin(RB_DIR_pin,direction[1]);
	set_pin(LB_DIR_pin,direction[2]);
	set_pin(LF_DIR_pin,direction[3]);
}
