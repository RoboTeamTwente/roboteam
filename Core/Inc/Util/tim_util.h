/*
 * tim_util.h
 *
 *  Created on: 8 April 2019
 *      Author: Cas Doornkamp
 */

#ifndef UTILS_TIM_UTIL_H_
#define UTILS_TIM_UTIL_H_

#include "stm32f7xx_hal.h"

// Timers (clocks still need to be defined: wheel_control, KickChip, Geneva)
extern TIM_HandleTypeDef htim1;		// encoder RF		16 bit
extern TIM_HandleTypeDef htim2;		// encoder Geneva
extern TIM_HandleTypeDef htim3;		// encoder LB		16 bit
extern TIM_HandleTypeDef htim4;		// encoder RB		16 bit
extern TIM_HandleTypeDef htim5;		// encoder LF		32 bit
extern TIM_HandleTypeDef htim6;		// Geneva
extern TIM_HandleTypeDef htim7;		// Control
extern TIM_HandleTypeDef htim8;		// PWM (CH1: RB, CH2: RF, CH3: Dribbler, CH4: Geneva)
extern TIM_HandleTypeDef htim9;		// PWM (CH1: LF, CH2: LB)
extern TIM_HandleTypeDef htim10;	// PWM (Buzzer)
extern TIM_HandleTypeDef htim11;	// KickChipping
extern TIM_HandleTypeDef htim12;	// internal timer (not used ATM)
extern TIM_HandleTypeDef htim13;	// internal timer (not used ATM)
extern TIM_HandleTypeDef htim14;	// internal timer (not used ATM)


// structs
typedef struct PWM_struct{
	TIM_HandleTypeDef* TIM;
	uint16_t Channel;
} PWM_struct;

// Timer Helpers
// Encoders
#define ENC_RF 		&htim1
#define ENC_GENEVA	&htim2
#define ENC_LB 		&htim3
#define ENC_RB 		&htim4
#define ENC_LF 		&htim5
#define TIM_SHOOT	&htim11
#define TIM_CONTROL &htim7
#define TIM_GENEVA  &htim6


extern PWM_struct PWM_RF;
extern PWM_struct PWM_RB;
extern PWM_struct PWM_LF;
extern PWM_struct PWM_LB;
extern PWM_struct PWM_Geneva;
extern PWM_struct PWM_Buzzer;
extern PWM_struct PWM_Dribbler;



inline void start_PWM(PWM_struct PWM){
	HAL_TIM_PWM_Start(PWM.TIM, PWM.Channel);
}

inline void stop_PWM(PWM_struct PWM){
	HAL_TIM_PWM_Stop(PWM.TIM, PWM.Channel);
}

inline void set_PWM(PWM_struct PWM, uint32_t Value){
	__HAL_TIM_SET_COMPARE(PWM.TIM , PWM.Channel, Value);
}

inline uint32_t get_PWM(PWM_struct PWM){
	return __HAL_TIM_GET_COMPARE(PWM.TIM , PWM.Channel);
}

#endif //UTILS_TIM_UTIL_H_
