/*
 * Geneva.c
 *
 *  Created on: Nov 9, 2018
 *      Author: kjhertenberg
 */

#include "Geneva.h"

///////////////////////////////////////////////////// DEFINITIONS

#define GENEVA_CAL_EDGE_CNT 1980	// the amount of counts from an edge to the center
#define GENEVA_CAL_SENS_CNT 1400	// the amount of counts from the sensor to the center
#define GENEVA_POSITION_DIF_CNT 810	// amount of counts between each of the five geneva positions
#define GENEVA_MAX_ALLOWED_OFFSET 0.2*GENEVA_POSITION_DIF_CNT	// maximum range where the geneva drive is considered in positon
#define SWITCH_OFF_TRESHOLD 200
#define MAX_DUTY_CYCLE_INVERSE_FRACTION 4

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
/*	to be called when the edge is detected
 * param:	the current distance to the middle positon
 */
static void geneva_EdgeCallback(int edge_cnt);

/*	check if the geneva drive got stuck and responds appropriatly
 *	param:
 *		dir: direction to go if we got stuck
 */
static void CheckIfStuck(int8_t dir);

static int32_t ClipInt(int32_t input, int32_t min, int32_t max);

static int32_t ClipFloat(float input, float min, float max);

// directly set the current output, if the pid control loop is running, this will not have much effect
static void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc);

// gives the raw encoder data
static int geneva_Encodervalue();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS
void geneva_Init(){
	geneva_state = geneva_setup;	// go to setup
	//initialize Pid values that have to stay initialized/declared
	Geneva_pid.PIDvar.kP = 50.0F; //kp
	Geneva_pid.PIDvar.kI = 4.0F; //ki
	Geneva_pid.PIDvar.kD = 0.7F; //kd
	Geneva_pid.PIDvar.prev_e = 0; //Always start at zero
	Geneva_pid.ref = 0.0F;
	Geneva_pid.actuator = &htim10;
	Geneva_pid.actuator_channel = TIM_CHANNEL_1;
	Geneva_pid.CallbackTimer = &htim6;
	Geneva_pid.CLK_FREQUENCY = 48000000.0F;
	Geneva_pid.dir[0] = Geneva_dir_B_Pin; // pin number of channel B
	Geneva_pid.dir[1] = Geneva_dir_A_Pin;// pin number of channel A
	Geneva_pid.dir_Port[0] = Geneva_dir_B_GPIO_Port; // GPIO Port of channel B
	Geneva_pid.dir_Port[1] = Geneva_dir_A_GPIO_Port; // GPIO Port of channel A
	Geneva_pid.max_pwm = Geneva_pid.actuator->Init.Period;
	HAL_TIM_PWM_Start(Geneva_pid.actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Start_IT(Geneva_pid.CallbackTimer);
	Geneva_pid.PIDvar.timeDiff = (((float)Geneva_pid.CallbackTimer->Init.Period))/(Geneva_pid.CLK_FREQUENCY/((float)(Geneva_pid.CallbackTimer->Init.Prescaler + 1)));
	__HAL_TIM_SET_AUTORELOAD(Geneva_pid.CallbackTimer, __HAL_TIM_GET_AUTORELOAD(Geneva_pid.CallbackTimer));
	HAL_TIM_Base_Start(&htim2);		// start the encoder
	geneva_cnt  = HAL_GetTick();	// store the start time
}

void geneva_Deinit(){
	HAL_TIM_Base_Start(&htim2);		// stop encoder
	HAL_TIM_PWM_Stop(Geneva_pid.actuator,TIM_CHANNEL_1);
	HAL_TIM_Base_Stop(Geneva_pid.CallbackTimer);
	geneva_state = geneva_idle;		// go to idle state
}

void geneva_Callback(){
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
	if(geneva_idle != geneva_state){
		float state = geneva_Encodervalue();
		float err = Geneva_pid.ref- state;
		if(abs(err)>GENEVA_MAX_ALLOWED_OFFSET*0.5){ //prevents the integrate control from oscillating around zero, and heating the driver
			float PIDoutput = PID(err, Geneva_pid.PIDvar); //PID from control_util.h
			PIDoutput = ClipFloat(PIDoutput, -4 * Geneva_pid.max_pwm, 4 * Geneva_pid.max_pwm); //Limit the PID output, legacy, not sure if necessary
			pid_SetOutput(PIDoutput, &Geneva_pid); //send signal to the motor
		}
	}
}

//Sets the ref based on the inputed value, called also by main
void geneva_SetPosition(geneva_positions position){
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
	return;
}

//called by putty
void geneva_SetState(geneva_states state){
	geneva_state = state;
}

//called by putty
geneva_positions geneva_GetPosition(){
	if((geneva_Encodervalue() % GENEVA_POSITION_DIF_CNT) > GENEVA_MAX_ALLOWED_OFFSET){
		return geneva_none;
	}
	return 2 + (geneva_Encodervalue()/GENEVA_POSITION_DIF_CNT);
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
// gives the raw encoder data
static int geneva_Encodervalue(){
	return (int32_t)__HAL_TIM_GetCounter(&htim2);
}

/*	to be called when the edge is detected
 * param:	the current distance to the middle positon
 */
static void geneva_EdgeCallback(int edge_cnt){
	__HAL_TIM_SET_COUNTER(&htim2, edge_cnt);
	Geneva_pid.ref = 0;
	geneva_state = geneva_returning;
}

/*	check if the geneva drive got stuck and responds appropriatly
 *	param:
 *		dir: direction to go if we got stuck
 */
static void CheckIfStuck(int8_t dir){
	static uint tick = 0xFFFF;			//
	static int enc;
	if(geneva_Encodervalue() != enc){
		enc = geneva_Encodervalue();
		tick = HAL_GetTick();
	}else if(tick + 70 < HAL_GetTick()){
		geneva_EdgeCallback(dir*GENEVA_CAL_EDGE_CNT);
	}
}

static int32_t ClipFloat(float input, float min, float max){
	return (input > max) ? max : (input < min) ? min : input;
}

// directly set the current output, if the pid control loop is running, this will not have much effect
static void pid_SetOutput(int pwm, PID_controller_HandleTypeDef* pc){
	if(pwm < -SWITCH_OFF_TRESHOLD){
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 1);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 0);
	}else if(pwm > SWITCH_OFF_TRESHOLD){
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 0);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 1);
	}else{
		HAL_GPIO_WritePin(pc->dir_Port[0], pc->dir[0], 0);
		HAL_GPIO_WritePin(pc->dir_Port[1], pc->dir[1], 0);
	}
	pwm = abs(pwm);
	pwm = ClipInt(pwm, 0, pc->actuator->Init.Period/MAX_DUTY_CYCLE_INVERSE_FRACTION);// Power limited by having maximum duty cycle, legacy not sure if necessary
	__HAL_TIM_SET_COMPARE(pc->actuator, pc->actuator_channel, pwm);
}

static int32_t ClipInt(int32_t input, int32_t min, int32_t max){
	return (input > max) ? max : (input < min) ? min : input;
}


