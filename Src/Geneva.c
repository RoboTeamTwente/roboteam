/*
 * Geneva.c
 *
 *  Created on: Nov 9, 2018
 *      Author: kjhertenberg
 */

#include "geneva.h"
#include "pid/pid.h"
#include "../PuttyInterface/PuttyInterface.h"

#define USE_SENSOR 0			// use the geneva sensor or always move to the edge
#define GENEVA_CAL_EDGE_CNT 1980	// the amount of counts from an edge to the center
#define GENEVA_CAL_SENS_CNT 1400	// the amount of counts from the sensor to the center
#define GENEVA_POSITION_DIF_CNT 810	// amount of counts between each of the five geneva positions
#define GENEVA_MAX_ALLOWED_OFFSET 0.2*GENEVA_POSITION_DIF_CNT	// maximum range where the geneva drive is considered in positon

geneva_states geneva_state = geneva_idle;	// current state of the geneva system

uint geneva_cnt;							// last measured encoder count

//declare de pid controller
PID_controller_HandleTypeDef Geneva_pid = {
		.pid = {0,0,0},
		.K_terms = {50.0F, 4.0F, 0.7F},
		.ref = 0.0F,
		.timestep = 0.0F,
		.actuator = &htim10,
		.actuator_channel = TIM_CHANNEL_1,
		.CallbackTimer = &htim6,
		.CLK_FREQUENCY = 48000000.0F,
		.current_pwm = 0,
		.dir[0] = Geneva_dir_B_Pin,
		.dir[1] = Geneva_dir_A_Pin,
		.dir_Port[0] = Geneva_dir_B_GPIO_Port,
		.dir_Port[1] = Geneva_dir_A_GPIO_Port,
};

/*--------------static functions---------------*/
/*	to be called when the edge is detected
 * param:	the current distance to the middle positon
 */
static void geneva_EdgeCallback(int edge_cnt){
	uprintf("ran into edge.\n\r");
	__HAL_TIM_SET_COUNTER(&htim2, edge_cnt);
	Geneva_pid.ref = 0;
	geneva_state = geneva_returning;
}

/*	check if the geneva drive got stuck and responds appropriatly
 *	param:
 *		dir: direction to go if we got stuck
 */
static inline void CheckIfStuck(int8_t dir){
	static uint tick = 0xFFFF;			//
	static int enc;
	if(geneva_Encodervalue() != enc){
		enc = geneva_Encodervalue();
		tick = HAL_GetTick();
	}else if(tick + 70 < HAL_GetTick()){
		geneva_EdgeCallback(dir*GENEVA_CAL_EDGE_CNT);
	}
}
/*---------------public functions-------------------*/
void geneva_Init(){
	geneva_state = geneva_setup;	// go to setup
	pid_Init(&Geneva_pid);			// initialize the pid controller
	HAL_TIM_Base_Start(&htim2);		// start the encoder
	geneva_cnt  = HAL_GetTick();	// store the start time
}

void geneva_Deinit(){
	HAL_TIM_Base_Start(&htim2);		// stop encoder
	pid_Deinit(&Geneva_pid);		// stop the pid controller
	geneva_state = geneva_idle;		// go to idle state
}

void geneva_Update(){
	switch(geneva_state){
	case geneva_idle:
		break;
	case geneva_setup:								// While in setup, slowly move towards the sensor/edge
		Geneva_pid.ref = (HAL_GetTick() - geneva_cnt)*1;	// if sensor is not seen yet, move to the right at 1 count per millisecond
		CheckIfStuck(1);
		break;
	case geneva_returning:					// while returning move to the middle position
		if(geneva_GetPosition() == geneva_middle){
			geneva_state = geneva_running;
		}else{
			geneva_SetPosition(geneva_middle);
		}
		break;
	case geneva_running:					// wait for external sources to set a new ref
		break;
	}
}

void geneva_Control(){
	if(geneva_idle != geneva_state){
		pid_Control(geneva_Encodervalue(), &Geneva_pid);
	}
}

int geneva_Encodervalue(){
	return (int32_t)__HAL_TIM_GetCounter(&htim2);
}

void geneva_SetState(geneva_states state){
	geneva_state = state;
}

geneva_states geneva_GetState(){
	return geneva_state;
}

geneva_positions geneva_SetPosition(geneva_positions position){
	switch(position){
	case geneva_rightright:
		Geneva_pid.ref = 2 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_right:
		Geneva_pid.ref = 1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_middle:
		Geneva_pid.ref = 0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_left:
		Geneva_pid.ref = -1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_leftleft:
		Geneva_pid.ref = -2 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_none:
		break;
	}
	return geneva_GetPosition();
}

geneva_positions geneva_GetPosition(){
	if((geneva_Encodervalue() % GENEVA_POSITION_DIF_CNT) > GENEVA_MAX_ALLOWED_OFFSET){
		return geneva_none;
	}
	return 2 + (geneva_Encodervalue()/GENEVA_POSITION_DIF_CNT);
}
