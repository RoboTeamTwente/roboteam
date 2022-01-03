#include "main.h"
#include "wheels.h"
#include "stdlib.h"

///////////////////////////////////////////////////// STRUCTS

static PIDvariables wheelsK[4];

///////////////////////////////////////////////////// VARIABLES

static bool wheels_initialized = false;
static bool wheels_braking = true;

static float wheels_measured_speeds[4] = {};      // Stores most recent measurement of wheel speeds in rad/s
static float wheels_commanded_speeds[4] = {};     // Holds most recent commanded wheel speeds in rad/s
static bool wheels_commanded_directions[4] = {};  // Holds most recent commanded wheel direction. 0 = CCW, 1 = CW
static uint32_t wheel_pwms[4] = {0};              // Holds the most recent wheel PWMs

// Mappings from wheels to GPIO encoder pins
static GPIO_Pin encoder_pins_A[4];
static GPIO_Pin encoder_pins_B[4];

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Reads out the value of the wheel encoders
static void getEncoderData(int16_t output_array[4]);

// Resets the encoders
static void resetWheelEncoders();

// Computes the speed of the wheels in rad/s using the encoder values
static void computeWheelSpeeds(float* speeds);

// Set the PWM for the wheels
static void setWheelPWMs(uint32_t* pwms);

// Clamps the PWM
static void limitWheelPWMs(uint32_t* pwms);

// Sets the direction of the wheels
static void setWheelDirections(bool* directions);





///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int wheels_Init(){

	/* Brake on startup */
	wheels_Brake();

	/* Initialize wheel controllers */
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		initPID(&wheelsK[wheel], 7.0, 0.0, 0.0);
	}

	/* Set PWM of wheels to 0, to prevent robot from suddenly shooting forward */
	/* Should never happen, but can't hurt */
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheel_pwms[wheel] = 0;
	}
	setWheelPWMs(wheel_pwms);

	/* Start the encoders */
	HAL_TIM_Base_Start(ENC_RF); //RF
	HAL_TIM_Base_Start(ENC_RB); //RB
	HAL_TIM_Base_Start(ENC_LB); //LB
	HAL_TIM_Base_Start(ENC_LF); //LF

	/* Start the PWM timers */
	start_PWM(PWM_RF); //RF
	start_PWM(PWM_RB); //RB
	start_PWM(PWM_LB); //LB
	start_PWM(PWM_LF); //LF


	/* Map encoder pins to GPIO pins */
	/* Currently unused, but can be used to manually read out an specific encoder channel */
	encoder_pins_A[wheels_RF] = RF_ENC_A_pin;
	encoder_pins_B[wheels_RF] = RF_ENC_B_pin;

	encoder_pins_A[wheels_RB] = RB_ENC_A_pin;
	encoder_pins_B[wheels_RB] = RB_ENC_B_pin;

	encoder_pins_A[wheels_LB] = LB_ENC_A_pin;
	encoder_pins_B[wheels_LB] = LB_ENC_B_pin;

	encoder_pins_A[wheels_LF] = LF_ENC_A_pin;
	encoder_pins_B[wheels_LF] = LF_ENC_B_pin;

	wheels_initialized = true;
	
	return 0;
}

int wheels_DeInit(){
	wheels_initialized = false;

	HAL_TIM_Base_Stop(ENC_RF); //RF
	HAL_TIM_Base_Stop(ENC_RB); //RB
	HAL_TIM_Base_Stop(ENC_LB); //LB
	HAL_TIM_Base_Stop(ENC_LF); //LF
	stop_PWM(PWM_RF); //RF
	stop_PWM(PWM_RB); //RB
	stop_PWM(PWM_LB); //LB
	stop_PWM(PWM_LF); //LF
	
	wheels_Stop();
	wheels_Brake();
	
	return 0;
}

void wheels_Update(){
	/* Don't run the wheels if these are not initialized */
	/* Not that anything would happen anyway, because the PWM timers wouldn't be running, but still .. */
	if(!wheels_initialized) return;

	/* Calculate the speeds of each wheel by looking at the encoders */
	computeWheelSpeeds(wheels_measured_speeds);
	
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		// Calculate the velocity error
		float angular_velocity_error = wheels_commanded_speeds[wheel] - wheels_measured_speeds[wheel];

		// If the error is very small, ignore it (why is this here?)
		if (fabs(angular_velocity_error) < 0.1) {
			angular_velocity_error = 0.0;
			wheelsK[wheel].I = 0;
		}

		// add PID to command angular velocity and convert to pwm
		wheel_pwms[wheel] = OMEGAtoPWM * (wheels_commanded_speeds[wheel] + PID(angular_velocity_error, &wheelsK[wheel])); 

		// Determine direction and if pwm is negative, switch directions
		// PWM < 0 : CounterClockWise. Direction = 0
		// 0 < PWM : ClockWise. Direction = 1
		wheels_commanded_directions[wheel] = 0 <= wheel_pwms[wheel];
		wheel_pwms[wheel] = abs(wheel_pwms[wheel]);
	}

	setWheelDirections( wheels_commanded_directions );
	setWheelPWMs( wheel_pwms );
	
}

void wheels_SetSpeeds(float speeds[4]){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheels_commanded_speeds[wheel] = speeds[wheel];
	}
}

void wheels_GetMeasuredSpeeds(float* speeds) {
	// Copy into "speeds", so that the file-local variable "wheels_measured_speeds" doesn't escape
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		speeds[wheel] = wheels_measured_speeds[wheel];
	}
}

void wheels_GetPWM(uint32_t* pwms) {
	pwms[wheels_RF] = get_PWM(PWM_RF);
	pwms[wheels_RB] = get_PWM(PWM_RB);
	pwms[wheels_LB] = get_PWM(PWM_LB);
	pwms[wheels_LF] = get_PWM(PWM_LF);
}

bool wheels_GetWheelsBraking() {
	return wheels_braking;
}

void wheels_Brake() {
	// Set pin to LOW to brake
	set_Pin(RB_Brake_pin, false);
	set_Pin(LB_Brake_pin, false);
	set_Pin(RF_Brake_pin, false);
	set_Pin(LF_Brake_pin, false);

	wheels_braking = true;
}

void wheels_Unbrake(){
	// Set pin to HIGH to unbrake
	set_Pin(RB_Brake_pin, true);
	set_Pin(LB_Brake_pin, true);
	set_Pin(RF_Brake_pin, true);
	set_Pin(LF_Brake_pin, true);

	wheels_braking = false;
}

/* Stop the motors without deinitializing them */
void wheels_Stop(){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheel_pwms[wheel] = 0;
	}
	setWheelPWMs(wheel_pwms);
}





///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void getEncoderData(int16_t* output_array){
	output_array[wheels_RF] = __HAL_TIM_GET_COUNTER(ENC_RF);
	output_array[wheels_RB] = __HAL_TIM_GET_COUNTER(ENC_RB);
	output_array[wheels_LB] = __HAL_TIM_GET_COUNTER(ENC_LB);
	output_array[wheels_LF] = __HAL_TIM_GET_COUNTER(ENC_LF);
}

static void resetWheelEncoders() {
	__HAL_TIM_SET_COUNTER(ENC_RF, 0);
	__HAL_TIM_SET_COUNTER(ENC_RB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LF, 0);
}

/**
 * @brief Calculates angular velocity in rad/s for each wheel based on their encoder values.
 * 
 * @param speeds float[4] in which the values will be placed.
 */
static void computeWheelSpeeds(float* speeds){
	int16_t encoder_values[4] = {0};
	getEncoderData(encoder_values);
	resetWheelEncoders();
	
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		// Convert encoder values to rad/s
		// We define clockwise as positive, therefore we have a minus sign here
		speeds[wheel] = -1 * ENCODERtoOMEGA * encoder_values[wheel]; 
	}	
}

/**
 * @brief Caps PWM values between PWM_CUTOFF and MAX_PWM
 * 
 * When the PWM for a wheel is below a certain level defined as PWM_CUTOFF, 
 * something bad happens. I don't actually know what happens. I assume the
 * power to the motor will be so low that the robot won't even move? It will
 * still consume power though, so it's better to just shut down the motor at
 * this point. 
 * @param pwms uint32_t[4] PWM values which will be capped
 */
static void limitWheelPWMs(uint32_t* pwms){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		if(pwms[wheel] < PWM_CUTOFF)  pwms[wheel] = 0;
		if(MAX_PWM     < pwms[wheel]) pwms[wheel] = MAX_PWM;
	}
}

static void setWheelPWMs(uint32_t* pwms){

	limitWheelPWMs(pwms);

	set_PWM(PWM_RF, pwms[wheels_RF]);
	set_PWM(PWM_RB, pwms[wheels_RB]);
	set_PWM(PWM_LB, pwms[wheels_LB]);
	set_PWM(PWM_LF, pwms[wheels_LF]);
}

static void setWheelDirections(bool* directions){
	set_Pin(RF_DIR_pin, directions[wheels_RF]);
	set_Pin(RB_DIR_pin, directions[wheels_RB]);
	set_Pin(LB_DIR_pin, directions[wheels_LB]);
	set_Pin(LF_DIR_pin, directions[wheels_LF]);
}
