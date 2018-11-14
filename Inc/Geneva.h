/*
 * Geneva.h
 *
 *  Created on: Nov 9, 2018
 *      Author: kjhertenberg
 */
/*
Description: controls the geneva drive

Instructions:
1) Initialize
2) it will calibrate
3) update the ref in main
4) apply PID control
5) set output

Extra functions:

GPIO Pins:
Geneva_dir_B_Pin; // pin number of channel B
Geneva_dir_A_Pin;// pin number of channel A
Geneva_dir_B_GPIO_Port; // GPIO Port of channel B
Geneva_dir_A_GPIO_Port; // GPIO Port of channel A

Notes:
Some code in here that i am unsure about if it's necessary
*/


#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_

#  if __has_include("stm32f0xx_hal.h")
#    include "stm32f0xx_hal.h"
#  elif  __has_include("stm32f3xx_hal.h")
#    include "stm32f3xx_hal.h"
#  elif  __has_include("stm32f4xx_hal.h")
#    include "stm32f4xx_hal.h"
#  endif

#include "tim.h"
#include "gpio.h"
#include "Utils/control_util.h"

#include <stdbool.h>
#include <stdlib.h>

///////////////////////////////////////////////////// VARIABLE STRUCT
//// Structs

typedef enum{
	geneva_idle,		// in idle it will do nothing
	geneva_setup,		// at startup it will try to find the edge sensor
	geneva_returning,	// when moving back to the initial/zero position
	geneva_running		// when being operational
}geneva_states;

typedef enum{
	geneva_leftleft,
	geneva_left,
	geneva_middle,
	geneva_right,
	geneva_rightright,
	geneva_none			// While rotating
}geneva_positions;

typedef struct{
	PIDvariables PIDvar;//struct from control_util.h
	float ref;
	TIM_HandleTypeDef* actuator;
	uint32_t actuator_channel;
	TIM_HandleTypeDef* CallbackTimer;
	float CLK_FREQUENCY;
	int16_t current_pwm;
	uint16_t dir[2];					// pin number of channel A and B respectively
	GPIO_TypeDef * dir_Port[2];			// GPIO Port of channel A and B respectively
	uint16_t max_pwm;
}PID_controller_HandleTypeDef;

PID_controller_HandleTypeDef Geneva_pid;
uint geneva_cnt;							// last measured encoder count
geneva_states geneva_state = geneva_idle;	// current state of the geneva system

///////////////////////////////////////////////////// FUNCTION PROTOTYPES
//// PUBLIC

void geneva_Init();

void geneva_Deinit();

/*	this function calls its pid controller and should be called with a specific time
 *
 */
void geneva_Callback();

/*	for debugging, sets the current geneva state
 *
 */
void geneva_SetState(geneva_states state);


/*	Set the position to one of the values of geneva_positions
 *
 */
void geneva_SetPosition(geneva_positions position);

/*	returns the current positions
 * 	from -2 to 2
 */
geneva_positions geneva_GetPosition();

#endif /* GENEVA_GENEVA_H_ */
