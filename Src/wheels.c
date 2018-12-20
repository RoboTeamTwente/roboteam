/*
 * wheels.c
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

#include "wheels.h"
#include <stdio.h>
#include <math.h>
#include "tim.h"


///////////////////////////////////////////////////// DEFINITIONS


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

static void getEncoderData(int encoderdata[4]);

static float deriveEncoder(int encoderData, int prev_encoderData);

static void limitScale();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void init(){
	wheels_state = wheels_ready;
	HAL_TIM_Base_Start(&htim1); //RF
	HAL_TIM_Base_Start(&htim8); //RB
	HAL_TIM_Base_Start(&htim3); //LB
	HAL_TIM_Base_Start(&htim4); //LF
	HAL_TIM_Base_Start(&htim5); //TIME
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); //RF
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); //RB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); //LB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); //LF
}

void deinit(){
	wheels_state = wheels_uninitialized;
	HAL_TIM_Base_Stop(&htim1); //RF
	HAL_TIM_Base_Stop(&htim8); //RB
	HAL_TIM_Base_Stop(&htim3); //LB
	HAL_TIM_Base_Stop(&htim4); //LF
	HAL_TIM_Base_Stop(&htim5); //TIME
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2); //RF
	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1); //RB
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1); //LB
	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2); //LF
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

static void getEncoderData(int encoderData[4]){
	encoderData[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim1);
	encoderData[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim8);
	encoderData[wheels_LB] = -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	encoderData[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim4);
}

static float deriveEncoder(int encoderData, int prev_encoderData){
	float wheel_speed = (encoderData-prev_encoderData)/TIME_DIFF;
	return wheel_speed;
}

static void SetPWM(int pwm[4]){
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, pwm[0]);
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, pwm[1]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, pwm[2]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, pwm[3]);
}

static void SetDir(bool direction[4]){
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, direction[0]);
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, direction[1]);
		HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, direction[2]);
		HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, direction[3]);
}
