
#include "../Inc/wheels.h"

///////////////////////////////////////////////////// STRUCTS

static PID_states wheels_state = off;
static PIDvariables wheelsK[4];

///////////////////////////////////////////////////// VARIABLES

static int pwm[4] = {0};
static bool direction[4] = {0}; // 0 is counter clock-wise
static float wheelSpeed[4] = {0};
static float wheelRef[4] = {0.0f};
static GPIO_Pin lockPins[4];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Reads out the value of the wheel encoders
static void getEncoderData(short int encoderdata[4]);

//Resets the encoder
static void ResetEncoder();

//Computes the speed of the wheels in rad/s using the encoder values
static void computeWheelSpeed();

//Clamps the PWM
static void limit();

//makes the PWM absolute
static void scale();

//Set the PWM for the wheels
static void SetPWM();

//Sets the direction of the wheels
static void SetDir();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int wheels_Init(){
	wheels_state = on;
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		initPID(&wheelsK[wheel], 7.0, 0.0, 0.0);
	}
	HAL_TIM_Base_Start(ENC_RF); //RF
	HAL_TIM_Base_Start(ENC_RB); //RB
	HAL_TIM_Base_Start(ENC_LB); //LB
	HAL_TIM_Base_Start(ENC_LF); //LF
	start_PWM(PWM_RF); //RF
	start_PWM(PWM_RB); //RB
	start_PWM(PWM_LB); //LB
	start_PWM(PWM_LF); //LF

	lockPins[wheels_RF] = RF_LOCK_pin;
	lockPins[wheels_RB] = RB_LOCK_pin;
	lockPins[wheels_LB] = LB_LOCK_pin;
	lockPins[wheels_LF] = LF_LOCK_pin;
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

	//TODO: Fix this huge stopping hack
	for (int i=0; i<4; i++) {
		pwm[i] = 0;
	}
	SetPWM();
	return 0;
}

void wheels_Update(){
	static uint lockTimes[4] = {0};
	if (wheels_state == on) {
		computeWheelSpeed();
		for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
			float err = wheelRef[wheel]-wheelSpeed[wheel];

			if (fabs(err) < 0.1) {
				err = 0.0;
				wheelsK[wheel].I = 0;
			}

			pwm[wheel] = OMEGAtoPWM*(wheelRef[wheel] + PID(err, &wheelsK[wheel])); // add PID to wheels reference angular velocity and convert to pwm
			if (read_Pin(lockPins[wheel])) {
				if (HAL_GetTick() - lockTimes[wheel] < 200) {
					pwm[wheel] = 0;
				}
			} else {
				lockTimes[wheel] = HAL_GetTick();
			}
		}

		scale();
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

static void scale(){
	static int Count[4] = {0};
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		if (Count[wheel] < 5){
			//for 50 ms the wheel cannot change direction
			//otherwise the motors drivers break
			if (direction[wheel]== 0 && pwm[wheel]<= -1.0F){
				pwm[wheel] *= -1;
			} else if (!(direction[wheel]==1 && pwm[wheel]>= 1.0F)){
				pwm[wheel] = 0;
			}
			Count[wheel] += 1;
		} else {
			// Determine direction
			if(pwm[wheel] <= -1.0F){
				pwm[wheel] *= -1;
				if (direction[wheel] == 1){
					Count[wheel] = 0;
				}
				direction[wheel] = 0; // turn anti-clockwise
			} else if(pwm[wheel] >= 1.0F){
				if (direction[wheel] == 0){
					Count[wheel] = 0;
				}
				direction[wheel] = 1; // turn clockwise
			}
		}
	}
}

static void limit(){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
	// Limit PWM
		if(pwm[wheel] < PWM_CUTOFF){
			pwm[wheel] = 0.0F;
		} else if(pwm[wheel] > MAX_PWM){
			pwm[wheel] = MAX_PWM;
		}
	}
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
