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

// Reads out the values of the wheel encoders
static void getEncoderData(int16_t output_array[4]);

// Resets the wheel encoders
static void resetWheelEncoders();

// Calculates angular velocity in rad/s for each wheel based on their encoder values
static void computeWheelSpeeds(float* speeds);

// Clamps PWM values between PWM_CUTOFF and MAX_PWM
static void limitWheelPWMs(uint32_t* pwms);

// Sets the wheel PWMs
static void setWheelPWMs(uint32_t* pwms);

// Sets the direction of the wheels
static void setWheelDirections(bool* directions);





///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

/**
 * @brief Initializes the PIDs / PWM timers / encoders
 */
void wheels_Init(){

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
	HAL_TIM_Base_Start(ENC_RF);
	HAL_TIM_Base_Start(ENC_RB);
	HAL_TIM_Base_Start(ENC_LB);
	HAL_TIM_Base_Start(ENC_LF);

	/* Start the PWM timers */
	start_PWM(PWM_RF);
	start_PWM(PWM_RB);
	start_PWM(PWM_LB);
	start_PWM(PWM_LF);

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

	// Set wheels to initialized
	wheels_initialized = true;
}

/**
 * @brief Deinitializes the PIDs / PWM timers / encoders
 */
void wheels_DeInit(){
	// Set wheels to uninitialized
	wheels_initialized = false;

	/* Stop the encoders */
	HAL_TIM_Base_Stop(ENC_RF);
	HAL_TIM_Base_Stop(ENC_RB);
	HAL_TIM_Base_Stop(ENC_LB);
	HAL_TIM_Base_Stop(ENC_LF);
	
	/* Stop the PWM timers */
	stop_PWM(PWM_RF);
	stop_PWM(PWM_RB);
	stop_PWM(PWM_LB);
	stop_PWM(PWM_LF);
	
	wheels_Stop();
	wheels_Brake();
}

/**
 * @brief Stops the wheels without deinitializing them 
 */
void wheels_Stop(){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheel_pwms[wheel] = 0;
	}
	setWheelPWMs(wheel_pwms);
}

/**
 * @brief Updates the wheels towards the commanded wheel speeds using the encoders and a PID controller.
 * 
 * This function is resonsible for getting the wheels to the commanded speeds, as stored in the file-local variable
 * "wheels_commanded_speeds". Wheel speeds, given in rad/s, are converted directly to a PWM value with help of the
 * conversion variable OMEGAtoPWM. This variable is based on information from the Maxon Motor datasheet. 
 * 
 * A PID controller is used to handle any error between the commanded and actual wheel speeds. First, the current wheel
 * speeds are measured by reading out the encoders and converting these values to rad/s. The commanded wheel speeds are
 * then subtracted from these measured wheel speeds, giving the error. This error is put through a PID controller, and
 * the resulting PID value is added to the commanded speeds before being converted to a PWM value. 
 * 
 * The resulting PWM values can be both positive and negative. This is split up into a "direction" boolean and a 
 * "PWN" integer. The "direction" boolean is false for CounterClockWise, and true for ClockWise. Finally both the 
 * directions and PWMs are sent to the wheels.
 */
void wheels_Update(){
	/* Don't run the wheels if these are not initialized */
	/* Not that anything would happen anyway, because the PWM timers wouldn't be running, but still .. */
	if(!wheels_initialized){
		wheels_Stop();
		return;
	}

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

		// Add PID to commanded speed and convert to PWM
		int32_t wheel_speed = OMEGAtoPWM * (wheels_commanded_speeds[wheel] + PID(angular_velocity_error, &wheelsK[wheel])); 

		// Determine direction and if pwm is negative, switch directions
		// PWM < 0 : CounterClockWise. Direction = 0
		// 0 < PWM : ClockWise. Direction = 1
		wheels_commanded_directions[wheel] = 0 <= wheel_speed;
		wheel_pwms[wheel] = abs(wheel_speed);
	}

	setWheelDirections( wheels_commanded_directions );
	setWheelPWMs( wheel_pwms );
}

/**
 * @brief Stores the commanded wheel speeds, in rad/s, to be used in the next wheels_Update() call
 * 
 * @param speeds float[4]{RF, RB, LB, LF} commanded wheels speeds, in rad/s. These values are stored in the file-local
 * variable 'wheels_commanded_speeds'. This variable will later be used in wheels_Update() to control the wheels.
 */
void wheels_SetSpeeds(const float* speeds){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		wheels_commanded_speeds[wheel] = speeds[wheel];
	}
}

/**
 * @brief Get the last measured wheel speeds in rad/s
 * 
 * @param speeds float[4]{RF, RB, LB, LF} output array in which the measured speeds will be stored
 */
void wheels_GetMeasuredSpeeds(float* speeds) {
	// Copy into "speeds", so that the file-local variable "wheels_measured_speeds" doesn't escape
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		speeds[wheel] = wheels_measured_speeds[wheel];
	}
}

/**
 * @brief Get the current wheel PWMs
 * 
 * @param pwms uint32_t[4]{RF, RB, LB, LF} output array in which the wheel PWMs will be stored
 */
void wheels_GetPWM(uint32_t* pwms) {
	pwms[wheels_RF] = get_PWM(PWM_RF);
	pwms[wheels_RB] = get_PWM(PWM_RB);
	pwms[wheels_LB] = get_PWM(PWM_LB);
	pwms[wheels_LF] = get_PWM(PWM_LF);
}

/**
 * @brief Get the current status of the brakes
 * 
 * @return true Wheels are braking
 * @return false Wheels are not braking
 */
bool wheels_GetWheelsBraking() {
	return wheels_braking;
}

/**
 * @brief Enable the brakes
 */
void wheels_Brake() {
	// Set pin to LOW to brake
	set_Pin(RB_Brake_pin, false);
	set_Pin(LB_Brake_pin, false);
	set_Pin(RF_Brake_pin, false);
	set_Pin(LF_Brake_pin, false);

	wheels_braking = true;
}

/**
 * @brief Disable the brakes
 */
void wheels_Unbrake(){
	// Set pin to HIGH to unbrake
	set_Pin(RB_Brake_pin, true);
	set_Pin(LB_Brake_pin, true);
	set_Pin(RF_Brake_pin, true);
	set_Pin(LF_Brake_pin, true);

	wheels_braking = false;
}





///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

/**
 * @brief Reads out the values of the wheel encoders
 * 
 * @param output_array int16_t[4]{RF, RB, LB, LF} output array in which the current encoder values will be placed
 */
static void getEncoderData(int16_t* output_array){
	output_array[wheels_RF] = __HAL_TIM_GET_COUNTER(ENC_RF);
	output_array[wheels_RB] = __HAL_TIM_GET_COUNTER(ENC_RB);
	output_array[wheels_LB] = __HAL_TIM_GET_COUNTER(ENC_LB);
	output_array[wheels_LF] = __HAL_TIM_GET_COUNTER(ENC_LF);
}

/**
 * @brief Resets the wheel encoders
 */
static void resetWheelEncoders() {
	__HAL_TIM_SET_COUNTER(ENC_RF, 0);
	__HAL_TIM_SET_COUNTER(ENC_RB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LB, 0);
	__HAL_TIM_SET_COUNTER(ENC_LF, 0);
}

/**
 * @brief Calculates angular velocity in rad/s for each wheel based on their encoder values
 * 
 * @todo This function requires to be called every 10 milliseconds, as dictated by the variable TIME_DIFF contained
 * within the variable ENCODERtoOMEGA. This can of course not always be perfectly guaranteed. Therefore, a timer should
 * be used to calculate the time difference between two calculations.
 * 
 * @param speeds float[4]{RF, RB, LB, LF} output array in which the calculated wheels speeds will be placed
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
 * @brief Clamps PWM values between PWM_CUTOFF and MAX_PWM
 * 
 * When the PWM for a wheel is below a certain level defined as PWM_CUTOFF, something bad happens. I don't actually
 * know what happens. I assume the power to the motor will be so low that the robot won't even move? It will still
 * consume power though, so it's better to just shut down the motor at this point. 
 * 
 * @param pwms uint32_t[4]{RF, RB, LB, LF} PWM values which will be clamped
 */
static void limitWheelPWMs(uint32_t* pwms){
	for(wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++){
		if(pwms[wheel] < PWM_CUTOFF)  pwms[wheel] = 0;
		if(MAX_PWM     < pwms[wheel]) pwms[wheel] = MAX_PWM;
	}
}

/**
 * @brief Sets the wheel PWMs
 * 
 * @param pwms uint32_t[4]{RF, RB, LB, LF} wheel PWMs
 */
static void setWheelPWMs(uint32_t* pwms){

	limitWheelPWMs(pwms);

	set_PWM(PWM_RF, pwms[wheels_RF]);
	set_PWM(PWM_RB, pwms[wheels_RB]);
	set_PWM(PWM_LB, pwms[wheels_LB]);
	set_PWM(PWM_LF, pwms[wheels_LF]);
}

/**
 * @brief Sets the direction of the wheels
 * 
 * @param directions bool[4]{RF, RB, LB, LF} wheel directions with false being CCW, true being CW
 */
static void setWheelDirections(bool* directions){
	set_Pin(RF_DIR_pin, directions[wheels_RF]);
	set_Pin(RB_DIR_pin, directions[wheels_RB]);
	set_Pin(LB_DIR_pin, directions[wheels_LB]);
	set_Pin(LF_DIR_pin, directions[wheels_LF]);
}
