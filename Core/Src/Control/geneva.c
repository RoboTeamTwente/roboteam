/*
 * geneva.c
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
 */

#include "geneva.h"
#include "../Util/control_util.h"

///////////////////////////////////////////////////// DEFINITIONS

#define GENEVA_CAL_EDGE_CNT 1980	// the amount of counts from an edge to the center
#define GENEVA_POSITION_DIF_CNT 810	// amount of counts between each of the five geneva positions
#define GENEVA_MAX_ALLOWED_OFFSET 0.2*GENEVA_POSITION_DIF_CNT	// maximum range where the geneva drive is considered in positon

///////////////////////////////////////////////////// VARIABLES

static PID_states geneva_state = off;	// current state of the geneva system
static float genevaRef = 0.0f;
static float pwm = 0.0f;
static PIDvariables genevaK = PIDdefault;

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

static void CheckIfStuck();

static int geneva_Encodervalue();

static void initPID(float kP, float kI, float kD);

static void setoutput(float pwm);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

//TODO: Fix PID channel and timer

void geneva_Init(){
	geneva_state = setup;	// go to setup
	initPID(50.0, 4.0, 0.7);		// initialize the pid controller
	HAL_TIM_Base_Start(&htim2);		// start the encoder
}

void geneva_Deinit(){
	HAL_TIM_Base_Start(&htim2);		// stop encoder
	geneva_state = off;		// go to idle state
}

void geneva_Update(){
	switch(geneva_state){
	case off:
		break;
	case setup:								// While in setup, slowly move towards the sensor/edge
		genevaRef = HAL_GetTick();	// if sensor is not seen yet, move to the right at 1 count per millisecond
		CheckIfStuck();
		break;
	case on:					// wait for external sources to set a new ref
		float err = genevaRef - geneva_Encodervalue();
		if (abs(err)>75){
			pwm = PID(err, genevaK);
		} else {
			pwm = 0;
		}
		//TODO: Scale function
		setoutput(pwm);
		break;
	}
}

geneva_positions geneva_SetPosition(geneva_positions position){
	switch(position){
	case geneva_rightright:
		genevaRef = 2 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_right:
		genevaRef = 1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_middle:
		genevaRef = 0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_left:
		genevaRef = -1 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_leftleft:
		genevaRef = -2 * GENEVA_POSITION_DIF_CNT;
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

int getPWM() {
	return pwm;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void initPID(float kP, float kI, float kD) {
		genevaK = PIDdefault;
		genevaK.kP = kP;
		genevaK.kI = kI;
		genevaK.kD = kD;
}

static void CheckIfStuck(){
	static uint tick = 0xFFFF;			//
	static int enc;
	if(geneva_Encodervalue() != enc){
		enc = geneva_Encodervalue();
		tick = HAL_GetTick();
	}else if(tick + 70 < HAL_GetTick()){
		__HAL_TIM_SET_COUNTER(&htim2, GENEVA_CAL_EDGE_CNT);
		genevaRef = 0;
		geneva_state = on;
	}
}


static int geneva_Encodervalue(){
	return (int32_t)__HAL_TIM_GetCounter(&htim2);
}

static void setoutput(float pwm){
	if(pwm < 0){
		HAL_GPIO_WritePin(Geneva_dir_B_GPIO_Port, Geneva_dir_B_Pin, 1);
		HAL_GPIO_WritePin(Geneva_dir_A_GPIO_Port, Geneva_dir_A_Pin, 0);
	}else if(pwm > 0){
		HAL_GPIO_WritePin(Geneva_dir_B_GPIO_Port, Geneva_dir_B_Pin, 0);
		HAL_GPIO_WritePin(Geneva_dir_A_GPIO_Port, Geneva_dir_A_Pin, 1);
	}else{
		HAL_GPIO_WritePin(Geneva_dir_B_GPIO_Port, Geneva_dir_B_Pin, 0);
		HAL_GPIO_WritePin(Geneva_dir_A_GPIO_Port, Geneva_dir_A_Pin, 0);
	}
	pwm = abs(pwm);
	int32_t currentPWM = ClipInt(pwm, 0, &htim10->Init.Period/MAX_DUTY_CYCLE_INVERSE_FRACTION);// Power limited by having maximum duty cycle
	__HAL_TIM_SET_COMPARE(&htim10, TIM_CHANNEL_1, currentPWM);
}
