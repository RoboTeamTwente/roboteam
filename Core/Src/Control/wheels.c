/*
 * wheels.c
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

//TODO: check includes
#include "wheels.h"
#include <stdio.h>
#include <math.h>
#include "tim_util.h"
#include "../Util/gpio_util.h"


///////////////////////////////////////////////////// VARIABLES

static PID_states wheels_state = off;
static int pwm[4] = {0};
static bool direction[4] = {0}; // 0 is counter clock-wise
static float wheelspeed[4] = {0};
static PIDvariables wheelsK[4];
static float wheelref[4] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Reads out the value of the wheel encoders
static void getEncoderData(short int encoderdata[4]);

//Resets the encoder
static void ResetEncoder();

//Computes the speed of the wheels in rad/s using the encoder values
static void computeWheelSpeed();

//Clamps the PWM
static void limitScale();

//Set the PWM for the wheels
static void SetPWM();

//Sets the direction of the wheels
static void SetDir();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int wheels_Init(){
	wheels_state = on;
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		initPID(wheelsK[wheel], 5.0, 0.0, 0.0);
	}
	HAL_TIM_Base_Start(&htim1); //RF
	HAL_TIM_Base_Start(&htim8); //RB
	HAL_TIM_Base_Start(&htim3); //LB
	HAL_TIM_Base_Start(&htim4); //LF
	HAL_TIM_Base_Start(&htim5); //TIME
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_2); //RF
	HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1); //RB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1); //LB
	HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_2); //LF
	return 0;
}

int wheels_Deinit(){
	wheels_state = off;
	HAL_TIM_Base_Stop(&htim1); //RF
	HAL_TIM_Base_Stop(&htim8); //RB
	HAL_TIM_Base_Stop(&htim3); //LB
	HAL_TIM_Base_Stop(&htim4); //LF
	HAL_TIM_Base_Stop(&htim5); //TIME
//	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_2); //RF
//	HAL_TIM_PWM_Stop(&htim9, TIM_CHANNEL_1); //RB
//	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_1); //LB
//	HAL_TIM_PWM_Stop(&htim12, TIM_CHANNEL_2); //LF

	//TODO: Fix this huge stopping hack
	for (int i=0; i<4; i++) {
		pwm[i] = 0;
	}
	SetPWM();
	return 0;
}

void wheels_Update(){
	if (wheels_state == on) {
		computeWheelSpeed();
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			float err = wheelref[wheel]-wheelspeed[wheel];
			pwm[wheel] = OMEGAtoPWM*(wheelref[wheel] + PID(err, &wheelsK[wheel])); // add PID to wheels reference angular velocity and convert to pwm
		}
		limitScale();
		SetDir();
		SetPWM();
	}
}

void wheels_SetRef(float input[4]){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheelref[wheel] = input[wheel];
	}
}

float wheels_GetState(wheel_names wheel) {
	return wheelspeed[wheel];
}

int wheels_GetPWM(wheel_names wheel) {
	return pwm[wheel];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void getEncoderData(short int encoderdata[4]){
	// NOTE: RF and RB are swapped to match with wheel reference
	encoderdata[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim8);
	encoderdata[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim1);
	encoderdata[wheels_LB] = __HAL_TIM_GET_COUNTER(&htim3);
	encoderdata[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim4);
}

static void ResetEncoder() {
	// NOTE: RF and RB are swapped to match with wheel reference
		__HAL_TIM_SET_COUNTER(&htim8, 0);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_SET_COUNTER(&htim3, 0);
		__HAL_TIM_SET_COUNTER(&htim4, 0);
}

static void computeWheelSpeed(){
	short int encoderData[4]= {0};
	getEncoderData(encoderData);
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheelspeed[wheel] = -1 * ENCODERtoOMEGA * encoderData[wheel]; // We define clockwise as positive, therefore we have a minus sign here
	}
	ResetEncoder();
}

static void limitScale(){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		// Determine direction
		if(pwm[wheel] <= -1.0F){
			pwm[wheel] *= -1;
			direction[wheel] = 0; // turn anti-clockwise
		} else if(pwm[wheel] >= 1.0F){
			direction[wheel] = 1; // turn clockwise
		} else {
			pwm[wheel] = 0; // the motor does not brake if pwm 0 is sent
		}
		// Limit PWM
		if(pwm[wheel] < PWM_CUTOFF){
			pwm[wheel] = 0.0F;
		} else if(pwm[wheel] > MAX_PWM){
			pwm[wheel] = MAX_PWM;
		}
	}
}

static void SetPWM(){
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, MAX_PWM-pwm[wheels_RF]);
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, MAX_PWM-pwm[wheels_RB]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, MAX_PWM-pwm[wheels_LB]);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, MAX_PWM-pwm[wheels_LF]);
}

static void SetDir(){
	HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, direction[wheels_RF]);
	HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, direction[wheels_RB]);
	HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, direction[wheels_LB]);
	HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, direction[wheels_LF]);
}
