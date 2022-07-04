#include "main.h"
#include "dribbler.h"
#include <math.h>

#define DRIBBLER_MAX_PWM 10000

movingAverage movingAvg = {0};

///////////////////////////////////////////////////// VARIABLES
static float dribbler_measured_speed = 0.0;             		   // Stores most recent measurement of dribbler speed in rad/s
static float dribbler_previous_measured_speed = 0.0;     		   // Stores the previous measurement of dribbler speed in rad/s
static float dribbler_filtered_measured_speed = 0.0; 		       // Stores filtered measurement of dribbler speed in rad/s
static float dribbler_previous_filtered_measured_speed = 0.0;      // Stores the previous filtered measurement of dribbler speed in rad/s
static bool hasBall = false;					        		   // Stores information if dribbler thinks it has the ball
static bool previousHadBall = false;				   			   // Stores the previous information about if dribbler thinks it has the ball

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Reads out the values of the wheel encoders
static void getEncoderData(int16_t *encoder_value);

// Resets the wheel encoders
static void resetWheelEncoders();

// Calculates angular velocity in rad/s for each wheel based on their encoder values
static void computeDribblerSpeed(float *speed);

// moving average filter on the dribbler speed
float smoothen_dribblerSpeed(float speed);

// Calculates the average of an array (buffer)
float getAvgOfBuffer(float *buffer, int size);

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
	int size = sizeof(movingAvg.commandedSpeedBuffer)/sizeof(movingAvg.commandedSpeedBuffer[0]);
    movingAvg.commandedSpeedBuffer[movingAvg.commandedIdx] = speed;
	movingAvg.commandedIdx = (movingAvg.commandedIdx+1) % size;	

	// The 12V and 24V boards require different calculations for the dribbler PWM
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	if (MOTORS_50W) {
		set_PWM(PWM_Dribbler, speed * DRIBBLER_MAX_PWM);
	} else {
		set_PWM(PWM_Dribbler, (1 - speed) * DRIBBLER_MAX_PWM);
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
	dribbler_previous_filtered_measured_speed = dribbler_filtered_measured_speed;
	computeDribblerSpeed(&dribbler_measured_speed);
	dribbler_filtered_measured_speed = smoothen_dribblerSpeed(dribbler_measured_speed);
}

/**
 * @brief Get the last measured wheel speeds in rad/s
 * 
 * @param speeds float[4]{RF, RB, LB, LF} output array in which the measured speeds will be stored
 */
void dribbler_GetMeasuredSpeeds(float *speed) {
	// Copy into "speed", so that the file-local variable "dribbler_measured_speed" doesn't escape
	*speed = dribbler_measured_speed;
}

void dribbler_GetFilteredSpeeds(float *speed) {
	// Copy into "speed", so that the file-local variable "dribbler_filtered_measured_speed" doesn't escape
	*speed = dribbler_filtered_measured_speed;
}

void dribbler_GetSpeedBeforeGotBall(float *speed) {
	// Copy into "speed", so that the file-local struct "movingAvg" doesn't escape
	*speed = movingAvg.speedBeforeGotBall;
}

bool dribbler_hasBall(){
	previousHadBall = hasBall; 

	// check if moving average is moving up or down (speed reduces when dribbler gets the ball)
	bool speed_reducing = ((dribbler_filtered_measured_speed  - dribbler_previous_filtered_measured_speed + 5) < 0); 
	bool speed_increasing = ((dribbler_filtered_measured_speed  - dribbler_previous_filtered_measured_speed) > 0);

	movingAvg.AvgCommandedSpeed = getAvgOfBuffer(movingAvg.commandedSpeedBuffer,sizeOfCommandBuffer);

	// update speed of the dribbler until it thinks it has the ball. This is used as the threshold value
	if (hasBall == false)
		// Check if data is in the reliable range
		if (movingAvg.movingAvgBuffer[(movingAvg.movingAvgIdx+(sizeOfMovingAverageBuffer-sizeOfDelay)) % sizeOfMovingAverageBuffer] > minReliableData)
			// Use a delayed value as the threshold (before it loses speed)
			movingAvg.speedBeforeGotBall = movingAvg.movingAvgBuffer[(movingAvg.movingAvgIdx+(sizeOfMovingAverageBuffer-sizeOfDelay)) % sizeOfMovingAverageBuffer];

	// check if all conditions are met, assume we have the ball if so
	if (speed_reducing && (dribbler_filtered_measured_speed > minReliableData) && (movingAvg.AvgCommandedSpeed > 0))
		hasBall = true;
	
	// Only say we lose the ball if the speed increases above the threshold value or if the dribbler turns off
	if (hasBall == true)
		if ((speed_increasing && (dribbler_filtered_measured_speed > (movingAvg.speedBeforeGotBall-5))) || movingAvg.AvgCommandedSpeed < 0.05)
			hasBall = false;
	
	return hasBall;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

/**
 * @brief Reads out the counts of the dribbler encoders
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
 * @todo This function requires to be called every 100 milliseconds, as dictated by the variable TIME_DIFF contained
 * within the variable ENCODERtoOMEGA. This can of course not always be perfectly guaranteed. Therefore, a timer should
 * be used to calculate the time difference between two calculations.
 */
static void computeDribblerSpeed(float *speed){
	int16_t encoder_value = 0;
	float measurement;
	getEncoderData(&encoder_value);
	resetWheelEncoders();
	
	// Convert encoder values to rad/s
	measurement = DRIBBLER_ENCODER_TO_OMEGA * (float) encoder_value;

	// for some reason the values are negative for some encoders and positive for others
	if (measurement < 0) 
		*speed = -measurement; 
	else
		*speed = measurement; 
}

float smoothen_dribblerSpeed(float speed){
    movingAvg.movingAvgBuffer[movingAvg.movingAvgIdx] = speed;
	movingAvg.movingAvgIdx = (movingAvg.movingAvgIdx+1) % sizeOfMovingAverageBuffer;

    return getAvgOfBuffer(movingAvg.movingAvgBuffer, sizeOfMovingAverageBuffer);
} 

float getAvgOfBuffer(float *buffer, int size){
	if (size == 0) return 0.0; // make sure we don't divide by zero
	
	float sum = 0.0;
	for (int i=0; i<size; i++){
        sum += buffer[i];
    }

	return sum/(float)size;
}