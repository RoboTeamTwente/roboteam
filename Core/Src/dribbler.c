#include "main.h"
#include "dribbler.h"


///////////////////////////////////////////////////// VARIABLES
static float dribbler_measured_speed;               // Stores most recent measurement of dribbler speed in rad/s
static float dribbler_previous_measured_speed;      // Stores the previous measurement of dribbler speed in rad/s
static bool hasBall = false;						// Stores information if dribbler thinks it has the ball
static bool previousHadBall = false;				// Stores the previous information about if dribbler thinks it has the ball

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Reads out the values of the wheel encoders
static void getEncoderData(int16_t *encoder_value);

// Resets the wheel encoders
static void resetWheelEncoders();

// Calculates angular velocity in rad/s for each wheel based on their encoder values
static void computeDribblerSpeed(float *speed);

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void dribbler_Init(){
	start_PWM(PWM_Dribbler);
	/* Start the encoder */
	HAL_TIM_Base_Start(ENC_DRIBBLER);
	dribbler_SetSpeed(0);
}

void dribbler_DeInit(){
	stop_PWM(PWM_Dribbler);
	/* Stop the encoder */
	HAL_TIM_Base_Stop(ENC_DRIBBLER);
}

void dribbler_SetSpeed(float speed){
	if(speed > 1){
		speed = 1;
	}else if(speed < 0){
		speed = 0;
	}

	// The 12V and 24V boards require different calculations for the dribbler PWM
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	if (MOTORS_50W) {
		// Dribbler is connected to same timer (htim8) as two motors, thus they share the same MAX_PWM
		set_PWM(PWM_Dribbler, speed * 10000);
	} else {
		set_PWM(PWM_Dribbler, (1 - speed) * 10000);
	}
}


/**
 * @brief Updates the dribbler towards the commanded dribbler speeds using the encoders and a PID controller.
 * 
 * This function is resonsible for getting the dribbler to the commanded speeds, as stored in the file-local variable
 * "dribbler_commanded_speed". 
 * 
 * A PID controller is used to handle any error between the commanded and actual dribbler speed. First, the current dribbler
 * speed is measured by reading out the encoder and converting that value to rad/s. The commanded dribbler speed is
 * then subtracted from these measured dribbler speed, giving the error. This error is put through a PID controller, and
 * the resulting PID value is added to the commanded speed before being converted to a PWM value. 
 */
void dribbler_Update(){
	dribbler_previous_measured_speed = dribbler_measured_speed;
	computeDribblerSpeed(&dribbler_measured_speed);
}

/**
 * @brief Get the last measured wheel speeds in rad/s
 * 
 * @param speeds float[4]{RF, RB, LB, LF} output array in which the measured speeds will be stored
 */
void dribbler_GetMeasuredSpeeds(float *speed) {
	// Copy into "speeds", so that the file-local variable "wheels_measured_speeds" doesn't escape
	*speed = dribbler_measured_speed;
}

bool dribbler_hasBall(){
	previousHadBall = hasBall;
	//hasBall = false;
	if (dribbler_measured_speed < 700.0 && ((dribbler_measured_speed - dribbler_previous_measured_speed + 5) < 0 ))
		hasBall = true;
	if (hasBall == true)
		if (dribbler_measured_speed > 900)
			hasBall = false;
	return hasBall;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

/**
 * @brief Reads out the values of the dribbler encoders
 * 
 * @param output_array int16_t[4]{RF, RB, LB, LF} output array in which the current encoder values will be placed
 */
static void getEncoderData(int16_t *encoder_value){
	*encoder_value = __HAL_TIM_GET_COUNTER(ENC_DRIBBLER);
}

/**
 * @brief Resets the wheel encoders
 */
static void resetWheelEncoders() {
	__HAL_TIM_SET_COUNTER(ENC_DRIBBLER, 0);
}

/**
 * @brief Calculates angular velocity in rad/s for the dribbler based on their encoder values
 * 
 * @todo This function requires to be called every 10 milliseconds, as dictated by the variable TIME_DIFF contained
 * within the variable ENCODERtoOMEGA. This can of course not always be perfectly guaranteed. Therefore, a timer should
 * be used to calculate the time difference between two calculations.
 * 
 * @param speeds float[4]{RF, RB, LB, LF} output array in which the calculated wheels speeds will be placed
 */
static void computeDribblerSpeed(float *speed){
	int16_t encoder_value = 0;
	getEncoderData(&encoder_value);
	resetWheelEncoders();
	
	// Convert encoder values to rad/s
	*speed = DRIBBLER_ENCODER_TO_OMEGA * encoder_value; 
}

