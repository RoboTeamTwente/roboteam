
#include "../Inc/geneva.h"

///////////////////////////////////////////////////// STRUCTS

static PID_states genevaState = off;
static PIDvariables genevaK;

///////////////////////////////////////////////////// VARIABLES

static float genevaRef = 0.0f;
static int pwm = 0;
static int direction[2] = {0};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Sets the encoder value to be zero at the edge of the robot
static void CheckIfStuck();

//Reads out the value of the encoder
static int geneva_Encodervalue();

//Clamps the PWM
static void limitScale();

//Sets the direction for the Geneva motor
static void setDir();

//Sets the PWM for the Geneva motor
static void setOutput();

// Checks if geneva is responding to pwm commands
static bool isResponding();

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void geneva_Init(){
	genevaState = setup;	// go to setup
	initPID(&genevaK, 10.0, 0.0, 0.0);
	HAL_TIM_Base_Start(ENC_GENEVA);		// start the encoder
	HAL_TIM_Base_Start_IT(TIM_GENEVA);
	start_PWM(PWM_Geneva);
}

void geneva_DeInit(){
	HAL_TIM_Base_Stop(ENC_GENEVA);		// stop encoder
	HAL_TIM_Base_Stop_IT(TIM_GENEVA);
	stop_PWM(PWM_Geneva);
	genevaState = off;		// go to idle state
}

void geneva_Update(){
	switch(genevaState){
	case off:
		return;
	case setup:								// While in setup, slowly move towards the sensor/edge
		genevaRef = 70000;	// if sensor is not seen yet, move to the right (70000 is above the max possible value)
		CheckIfStuck();
		break;
	case on: 				// wait for external sources to set a new ref
		break;
	}
	float err = genevaRef - geneva_Encodervalue();
	pwm = PID(err, &genevaK);
	limitScale();
	if (!isResponding()) {
		pwm = 0;
	}
	setDir();
	setOutput();
}

void geneva_SetRef(geneva_positions position){
	if (position != geneva_none) {
		genevaRef = encoderForPosition[position];
	}
}

float geneva_GetRef(){
	return genevaRef;
}

int geneva_GetEncoder(){
	return geneva_Encodervalue();
}

int geneva_GetPWM(){
	return pwm;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void CheckIfStuck(){
	static uint tick = 0xFFFF;			//
	static int enc = 0;
	if(fabs(geneva_Encodervalue() - enc) < ENCODER_DEVIATION_MARGIN){
		enc = geneva_Encodervalue();
		tick = HAL_GetTick();
	}else if(tick + 500 < HAL_GetTick()){
		__HAL_TIM_SET_COUNTER(ENC_GENEVA, GENEVA_CAL_EDGE_CNT);
		geneva_SetRef(geneva_middle);
		genevaState = on;
	}
}

static int geneva_Encodervalue(){
	return (int32_t)__HAL_TIM_GetCounter(ENC_GENEVA);
}

static void limitScale(){
	if(pwm < 0.0F){
		pwm *= -1;
		direction[0] = 1;
		direction[1] = 0;
	} else if(pwm > 0.0F){
		direction[0] = 0;
		direction[1] = 1;
	} else {
		pwm = 0;
		direction[0] = 0;
		direction[1] = 0;
	}
	// Limit PWM
	if(pwm < PWM_CUTOFF){
		pwm = 0.0F;
	} else if(pwm > MAX_PWM){
		pwm = MAX_PWM;
	}
}

static bool isResponding() {
	static bool result = true;
	static int cnt = 0;
	static int prevEncoder = 0;
	if ((pwm > 0) && (fabs(geneva_Encodervalue() - prevEncoder)) < ENCODER_DEVIATION_MARGIN)  {
		cnt++;
	} else {
		cnt = 0;
		result = true;
	}
	if (cnt >= 10) {
		result = false;
	}
	prevEncoder = geneva_Encodervalue();
	return result;
}

static void setDir(){
	set_Pin(Geneva_DIR_B_pin, direction[0]);
	set_Pin(Geneva_DIR_A_pin, direction[1]);
}

static void setOutput(){
	set_PWM(PWM_Geneva, pwm);
}
