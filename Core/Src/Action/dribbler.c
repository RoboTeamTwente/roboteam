/*
 * dribbler.c
 *
 *  Created on: May 23, 2018
 *      Author: Leon
 */

#include <Action/dribbler.h>

#include "tim.h"

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void dribbler_Init(){
	HAL_TIM_PWM_Start(&htim11, TIM_CHANNEL_1);
	dribbler_SetSpeed(0);
}

void dribbler_Deinit(){
	HAL_TIM_PWM_Stop(&htim11, TIM_CHANNEL_1);
}

void dribbler_SetSpeed(int speed){
	if(speed > 7){
		speed = 7;
	}else if(speed < 0){
		speed = 0;
	}
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, (7 - speed) * (MAX_PWM / 7));
}
