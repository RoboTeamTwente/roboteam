/*
 * wheels.c
 *
 *  Created on: Dec 4, 2018
 *      Author: kjhertenberg
 */

#include "wheels.h"

#include "tim.h"

///////////////////////////////////////////////////// DEFINITIONS



///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

void setOutput(int output[4]);

void getEncoderData(int encoderdata[4]);


void deriveEncoder();

void limitScale();

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
	bool reverse[4] = {0};
	wheelref[0] = 0;
	wheelref[1] = 0;
	wheelref[2] = 0;
	wheelref[3] = 0;
	switch(wheels_state){
	case wheels_uninitialized:
	//		uprintf("ERROR wheels_uninitialized\n\r");
			return;
	case wheels_ready:
		int encoderData[4] = {0};
		int output[4] = {0};

		//get encoder data
		getEncoderData(encoderData);

		//derive wheelspeed

		//PID

		//combine reference and PID

		limitScale(wheelref, output, reverse);

		SetDir(reverse);
		SetPWM(output);
		return;
	}
	return;
}



///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
void limitScale(float wheelref[4], float output[4], float reverse[4]){
	for(int i = wheels_RF; i <= wheels_LF; i++){
		if(wheelref[i] <= -1.0F){
			output[i] = -wheelref[i];
			reverse[i] = 1;
		}else if(wheelref[i] >= 1.0F){
			reverse[i] = 0;
		}
		if(wheelref[i] < 3){
			output[i] = 0.0F;
		}else if(wheelref[i] < 3.1){
			output[i] = 3.1;
		}else if(wheelref[i] > 100){
			output[i] = 100;
		}
	}
}

void getEncoderData(int encoderData[4]){
	encoderData[wheels_RF] = __HAL_TIM_GET_COUNTER(&htim1);
	encoderData[wheels_RB] = __HAL_TIM_GET_COUNTER(&htim8);
	encoderData[wheels_LB] = -__HAL_TIM_GET_COUNTER(&htim3); //  TODO: minus due to inverted routing (old robot)
	encoderData[wheels_LF] = __HAL_TIM_GET_COUNTER(&htim4);
}



/*	Set Pwm for one wheel
 *
 */
static inline void SetPWM(float power[4]){
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_2, power[0] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim9 , TIM_CHANNEL_1, power[1] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_1, power[2] / 100 * MAX_PWM);
	__HAL_TIM_SET_COMPARE(&htim12, TIM_CHANNEL_2, power[3] / 100 * MAX_PWM);
}
/*	Set direction for one wheel
 *
 */
static inline void SetDir(bool reverse[4]){
		HAL_GPIO_WritePin(FR_RF_GPIO_Port,FR_RF_Pin, reverse[0]);
		HAL_GPIO_WritePin(FR_RB_GPIO_Port,FR_RB_Pin, reverse[1]);
		HAL_GPIO_WritePin(FR_LB_GPIO_Port,FR_LB_Pin, reverse[2]);
		HAL_GPIO_WritePin(FR_LF_GPIO_Port,FR_LF_Pin, reverse[3]);
}
