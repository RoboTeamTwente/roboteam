
#include "../Inc/Control/geneva.h"

///////////////////////////////////////////////////// STRUCTS

static PID_states genevaState = off;
static PIDvariables genevaK;

///////////////////////////////////////////////////// VARIABLES

static float genevaRef = 0.0f;
static float pwm = 0.0f;
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

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

//TODO: Fix/check timers, channels and PINS
//TODO: Fix PID channel and timer

void geneva_Init(){
	genevaState = setup;	// go to setup
	initPID(genevaK, 50.0, 4.0, 0.7);		// initialize the pid controller
	HAL_TIM_Base_Start(ENC_GENEVA);		// start the encoder
}

void geneva_Deinit(){
	HAL_TIM_Base_Stop(ENC_GENEVA);		// stop encoder
	genevaState = off;		// go to idle state
}

void geneva_Update(){
	switch(genevaState){
	case off:
		break;
	case setup:								// While in setup, slowly move towards the sensor/edge
		genevaRef = HAL_GetTick();	// if sensor is not seen yet, move to the right at 1 count per millisecond
		CheckIfStuck();
		break;
	case on: ;				// wait for external sources to set a new ref // semicolon is an empty statement for C grammar does not allow a label directly after a :
		float err = genevaRef - geneva_Encodervalue();
		if (fabs(err)>GENEVA_MAX_ALLOWED_OFFSET){
			pwm = PID(err, &genevaK);
		} else {
			pwm = 0;
		}
		limitScale();
		setDir();
		setOutput();
		break;
	}
}

void geneva_SetRef(geneva_positions position){
	switch(position){
	case geneva_rightright:
		genevaRef = 2.0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_right:
		genevaRef = 1.0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_middle:
		genevaRef = 0.0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_left:
		genevaRef = -1.0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_leftleft:
		genevaRef = -2.0 * GENEVA_POSITION_DIF_CNT;
		break;
	case geneva_none:
		break;
	}
}

geneva_positions geneva_GetState(){
	if((geneva_Encodervalue() % GENEVA_POSITION_DIF_CNT) > GENEVA_MAX_ALLOWED_OFFSET){
		return geneva_none;
	}
	return 2 + (geneva_Encodervalue()/GENEVA_POSITION_DIF_CNT);
}

int geneva_GetPWM() {
	return pwm;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void CheckIfStuck(){
	static uint tick = 0xFFFF;			//
	static int enc;
	if(geneva_Encodervalue() != enc){
		enc = geneva_Encodervalue();
		tick = HAL_GetTick();
	}else if(tick + 70 < HAL_GetTick()){
		__HAL_TIM_SET_COUNTER(ENC_GENEVA, GENEVA_CAL_EDGE_CNT);
		genevaRef = 0;
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
	// Limit PWM //TODO: figure out MAX_PWM from old code
	if(pwm < PWM_CUTOFF){
		pwm = 0.0F;
	} else if(pwm > MAX_PWM){
		pwm = MAX_PWM;
	}
}

static void setDir(){
	set_Pin(Geneva_DIR_B_pin, direction[0]);
	set_Pin(Geneva_DIR_A_pin, direction[1]);
}

static void setOutput(){
	set_PWM(PWM_Geneva, pwm);
}
