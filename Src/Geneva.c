/*
 * Geneva.c
 *
 *  Created on: Nov 9, 2018
 *      Author: kjhertenberg
 */

#include "Geneva.h"


///////////////////////////////////////////////////// DEFINITIONS
#define GENEVA_CAL_EDGE_CNT 1980	// the amount of counts from an edge to the center
#define GENEVA_POSITION_DIF_CNT 810	// amount of counts between each of the five geneva positions
#define GENEVA_MAX_ALLOWED_OFFSET 0.2*GENEVA_POSITION_DIF_CNT	// maximum range where the geneva drive is considered in positon

geneva_states geneva_state = geneva_setup;	// current state of the geneva system

uint geneva_cnt;							// last measured encoder count

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Is called when we find the edge and calibrates the encoders values based on the position
static void geneva_EdgeCallback(int edge_cnt);

//checks if we found the edge
static inline void CheckIfStuck(int8_t dir);

static void geneva_Control();

static float geneva_SetRef(geneva_positions position);

static float getState();

static float singlePID(float ref, float state, float K[3]);

static float scaleAndLimit(float controlValue)

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void geneva_Init(){
	geneva_state = geneva_setup;	// go to setup
	HAL_TIM_Base_Start(&htim2);		// start the encoder
	geneva_cnt  = HAL_GetTick();	// store the start time
}

void geneva_Deinit(){
	HAL_TIM_Base_Start(&htim2);		// stop encoder
	geneva_state = geneva_idle;		// go to idle state
}

void geneva_Callback(int genevaStateRef){
	float genevaK[3] = {0,0,0};//kp,ki,kp
	static float genevaRef = 0;
	static int currentStateRef = 3; //impossible state to kick-start

	switch(geneva_state){
		case geneva_idle:
			return;
		case geneva_setup:								// While in setup, slowly move towards the sensor/edge
			genevaRef = (HAL_GetTick() - geneva_cnt)*1;	// if sensor is not seen yet, move to the right at 1 count per millisecond
			CheckIfStuck(1);
			break;
		case geneva_running:					// wait for external sources to set a new ref
			if (genevaStateRef != currentStateRef){
				genevaRef = geneva_SetRef(genevaStateRef);
				currentStateRef = genevaStateRef;
			}
			break;
	}

	geneva_Control(genevaRef, genevaK);

}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void geneva_EdgeCallback(int edge_cnt){
	uprintf("ran into edge.\n\r");
	__HAL_TIM_SET_COUNTER(&htim2, edge_cnt);
	geneva_state = geneva_running;
}

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

static void geneva_Control(float ref, float K[3]){

	float state = getState();
	float controlValue = singlePID(ref, state, K);
	float PWM = scaleAndLimit(controlValue);

	//set motor

	//pid_Control(geneva_Encodervalue(), &Geneva_pid);
}

static float geneva_SetRef(geneva_positions position){
	switch(position){
	case geneva_rightright:
		return 2 * GENEVA_POSITION_DIF_CNT;
	case geneva_right:
		return 1 * GENEVA_POSITION_DIF_CNT;
	case geneva_middle:
		return 0 * GENEVA_POSITION_DIF_CNT;
	case geneva_left:
		return -1 * GENEVA_POSITION_DIF_CNT;
	case geneva_leftleft:
		return -2 * GENEVA_POSITION_DIF_CNT;
	case geneva_none:
		break;
	}
	return 0;
}

static float getState(){
	return 0;
}

static float singlePID(float ref, float state, float K[3]){
	 return 0;
 }

static float scaleAndLimit(float controlValue){
	return 0;
}


