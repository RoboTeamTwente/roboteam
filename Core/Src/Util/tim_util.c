/*
 * tim_util.c
 *
 *  Created on: 8 April 2019
 *      Author: Cas Doornkamp
 */

#include "tim_util.h"
#include "stm32f7xx_hal.h"

PWM_struct PWM_RF		= { &htim8 , TIM_CHANNEL_2};
PWM_struct PWM_RB		= { &htim8 , TIM_CHANNEL_1};
PWM_struct PWM_LF		= { &htim9 , TIM_CHANNEL_1};
PWM_struct PWM_LB		= { &htim9 , TIM_CHANNEL_2};
PWM_struct PWM_Geneva	= { &htim8 , TIM_CHANNEL_4};
PWM_struct PWM_Buzzer	= { &htim10 , TIM_CHANNEL_1};
PWM_struct PWM_Dribbler	= { &htim8 , TIM_CHANNEL_3};
