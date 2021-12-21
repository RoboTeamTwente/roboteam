#include "main.h"
#include "wheels.h"
#include "stdlib.h"

///////////////////////////////////////////////////// STRUCTS

static PID_states wheels_state = off;
static PIDvariables wheelsK[4];

///////////////////////////////////////////////////// VARIABLES

static int pwm[4] = {0};
static bool direction[4] = {0}; // 0 is counter clock-wise
static float wheelSpeed[4] = {0};
static float wheelRef[4] = {0.0f};
static bool isBraking = false;

// Encoder
static GPIO_Pin encoderAPins[4];
static GPIO_Pin encoderBPins[4];
static int count[4] = {0};
static bool noEncoder[4] = {false};
static bool Aold[4];
static bool Bold[4];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Reads out the value of the wheel encoders
static void getEncoderData(short int encoderdata[4]);

//Resets the encoder
static void ResetEncoder();

//Computes the speed of the wheels in rad/s using the encoder values
static void computeWheelSpeed();

//Clamps the PWM
static void limit();

//Set the PWM for the wheels
static void SetPWM();

//Sets the direction of the wheels
static void SetDir();

//Checks whether the encoder is working correctly
static void checkEncoder(wheel_names wheel);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int wheels_Init(){
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	wheels_state = on;
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		if (MOTORS_50W) {
			initPID(&wheelsK[wheel], 7.0, 0.0, 0.0); // 50 W
		} else {
			initPID(&wheelsK[wheel], 7.0, 0.0, 0.0); // 30 W
		}
	}
	HAL_TIM_Base_Start(ENC_RF); //RF
	HAL_TIM_Base_Start(ENC_RB); //RB
	HAL_TIM_Base_Start(ENC_LB); //LB
	HAL_TIM_Base_Start(ENC_LF); //LF
	start_PWM(PWM_RF); //RF
	start_PWM(PWM_RB); //RB
	start_PWM(PWM_LB); //LB
	start_PWM(PWM_LF); //LF
	wheels_Brake(true); // Brake on startup

	encoderAPins[wheels_RF] = RF_ENC_A_pin;
	encoderAPins[wheels_RB] = RB_ENC_A_pin;
	encoderAPins[wheels_LB] = LB_ENC_A_pin;
	encoderAPins[wheels_LF] = LF_ENC_A_pin;
	encoderBPins[wheels_RF] = RF_ENC_B_pin;
	encoderBPins[wheels_RB] = RB_ENC_B_pin;
	encoderBPins[wheels_LB] = LB_ENC_B_pin;
	encoderBPins[wheels_LF] = LF_ENC_B_pin;

	return 0;
}

int wheels_DeInit(){
	wheels_state = off;
	HAL_TIM_Base_Stop(ENC_RF); //RF
	HAL_TIM_Base_Stop(ENC_RB); //RB
	HAL_TIM_Base_Stop(ENC_LB); //LB
	HAL_TIM_Base_Stop(ENC_LF); //LF
	stop_PWM(PWM_RF); //RF
	stop_PWM(PWM_RB); //RB
	stop_PWM(PWM_LB); //LB
	stop_PWM(PWM_LF); //LF
	wheels_Brake(true); // Brake

	//TODO: Fix this huge stopping hack
	for (int i=0; i<4; i++) {
		pwm[i] = 0;
	}
	SetPWM();
	return 0;
}


/* Stop the motors without deinitializing them */
void wheels_Stop(){
	for (int i=0; i<4; i++) {
		pwm[i] = 0;
	}
	SetPWM();
}

void wheels_Update(){
	if (wheels_state == on) {
		computeWheelSpeed();
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			// Check encoder data
			//checkEncoder(wheel);

			/*if (noEncoder[wheel]) {
				// Do not use PID when there is no encoder data
				pwm[wheel] = OMEGAtoPWM*wheelRef[wheel];
			} else {*/
			float err = wheelRef[wheel]-wheelSpeed[wheel];

			if (fabs(err) < 0.1) {
				err = 0.0;
				wheelsK[wheel].I = 0;
			}

			pwm[wheel] = OMEGAtoPWM*(wheelRef[wheel] + PID(err, &wheelsK[wheel])); // add PID to wheels reference angular velocity and convert to pwm
			
		}

		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			// Determine direction and if pwm is negative, switch directions
			// PWM < 0 : CounterClockWise. Direction = 0
			// 0 < PWM : ClockWise. Direction = 1
			direction[wheel] = 0 <= pwm[wheel];
			pwm[wheel] = abs(pwm[wheel]);
	}
		limit();
		SetDir();
		SetPWM();
	}
}

void wheels_SetRef(float input[4]){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheelRef[wheel] = input[wheel];
	}
}

float* wheels_GetState() {
	return wheelSpeed;
}

int* wheels_GetPWM() {
	return pwm;
}

bool wheels_IsBraking() {
	return isBraking;
}

void wheels_Brake(bool brake) {
	// Set pin to LOW to brake
	set_Pin(RB_Brake_pin, !brake);
	set_Pin(LB_Brake_pin, !brake);
	set_Pin(RF_Brake_pin, !brake);
	set_Pin(LF_Brake_pin, !brake);

	isBraking = brake;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void getEncoderData(short int encoderdata[4]){
	encoderdata[wheels_RF] = __HAL_TIM_GET_COUNTER(ENC_RF);
	encoderdata[wheels_RB] = __HAL_TIM_GET_COUNTER(ENC_RB);
	encoderdata[wheels_LB] = __HAL_TIM_GET_COUNTER(ENC_LB);
	encoderdata[wheels_LF] = __HAL_TIM_GET_COUNTER(ENC_LF);
}

static void ResetEncoder() {
	__HAL_TIM_SET_COUNTER(ENC_RF, 0);
	__HAL_TIM_SET_COUNTER(ENC_RB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LF, 0);
}

static void computeWheelSpeed(){
	short int encoderData[4]= {0};
	getEncoderData(encoderData);
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheelSpeed[wheel] = -1 * ENCODERtoOMEGA * encoderData[wheel]; // We define clockwise as positive, therefore we have a minus sign here
	}
	ResetEncoder();
}


static void SetPWM(){
	set_PWM(PWM_RF, pwm[wheels_RF]);
	set_PWM(PWM_RB, pwm[wheels_RB]);
	set_PWM(PWM_LB, pwm[wheels_LB]);
	set_PWM(PWM_LF, pwm[wheels_LF]);
}

static void SetDir(){
	set_Pin(RF_DIR_pin, direction[wheels_RF]);
	set_Pin(RB_DIR_pin, direction[wheels_RB]);
	set_Pin(LB_DIR_pin, direction[wheels_LB]);
	set_Pin(LF_DIR_pin, direction[wheels_LF]);
}

// This is not working properly
/*
static void checkEncoder(wheel_names wheel) {
	static const int threshold = 10; // Number of ticks the encoder data can be the same before detection of unconnected encoder

	bool A = read_Pin(encoderAPins[wheel]);
	bool B = read_Pin(encoderBPins[wheel]);

	// When pwm is larger than the cutoff pwm, but encoder is not moving, then there is no correct encoder data
	if (!noEncoder[wheel] && (A == Aold[wheel] || B == Bold[wheel]) && fabs(pwm[wheel]) >= PWM_CUTOFF) {
		if (count[wheel] >= threshold) {
			noEncoder[wheel] = true;
		} else {
			count[wheel]++;
		}
	} else {
		count[wheel] = 0;
	}

	// Check if encoder data is back
	if (noEncoder[wheel] && A != Aold[wheel] && B != Bold[wheel]) {
		noEncoder[wheel] = false;
	}

	Aold[wheel] = A;
	Bold[wheel] = B;
}
*/