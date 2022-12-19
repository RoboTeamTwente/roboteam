#include "dribbler.h"

movingAverage movingAvg = {0};

///////////////////////////////////////////////////// VARIABLES
static float dribbler_measured_speed = 0.0;             		   // Stores most recent measurement of dribbler speed in rad/s
static float dribbler_filtered_measured_speed = 0.0; 		       // Stores filtered measurement of dribbler speed in rad/s
static float dribbler_previous_filtered_measured_speed = 0.0;      // Stores the previous filtered measurement of dribbler speed in rad/s
static bool hasBall = false;					        		   // Stores information if dribbler thinks it has the ball

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Reads out the values of the wheel encoders
static int16_t getEncoderData();
// Resets the dribbler encoder
static void resetDribblerEncoders();
// Calculates angular velocity in rad/s for each wheel based on their encoder values
static void computeDribblerSpeed();
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

    movingAvg.commandedSpeedBuffer[movingAvg.commandedIdx] = speed;
	movingAvg.commandedIdx = (movingAvg.commandedIdx+1) % sizeOfCommandBuffer;	

	// The 12V and 24V boards require different calculations for the dribbler PWM
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	if (MOTORS_50W) {
		set_PWM(PWM_Dribbler, speed * DRIBBLER_MAX_PWM);
	} else {
		set_PWM(PWM_Dribbler, (1 - speed) * DRIBBLER_MAX_PWM);
	}
}


/**
 * @brief Updates the dribbler variables
 */
void dribbler_Update(){
	dribbler_previous_filtered_measured_speed = dribbler_filtered_measured_speed;
	computeDribblerSpeed();
	dribbler_filtered_measured_speed = smoothen_dribblerSpeed(dribbler_measured_speed);
}

/**
 * @brief Get the last measured dribbler speed in rad/s
 */
float dribbler_GetMeasuredSpeeds() {
	return dribbler_measured_speed;
}

/**
 * @brief Get the last filtered dribbler speed in rad/s
 */
float dribbler_GetFilteredSpeeds() {
	return dribbler_filtered_measured_speed;
}

/**
 * @brief Get the dribbler speed it had before getting the ball in rad/s
 */
float dribbler_GetSpeedBeforeGotBall() {
	return movingAvg.speedBeforeGotBall;
}

/**
 * @brief Estimate if dribbler has the ball or not
 */
bool dribbler_hasBall(){
	// check if moving average is moving up or down (speed reduces when dribbler gets the ball)
	bool speed_reducing = ((dribbler_filtered_measured_speed  - dribbler_previous_filtered_measured_speed + 5) < 0); 
	bool speed_increasing = ((dribbler_filtered_measured_speed  - dribbler_previous_filtered_measured_speed) > 0);

	float AvgCommandedSpeed = getAvgOfBuffer(movingAvg.commandedSpeedBuffer,sizeOfCommandBuffer);

	// update speed of the dribbler until it thinks it has the ball. This is used as the threshold value
	if (hasBall == false){
		// Check if data is in the reliable range
		if (movingAvg.movingAvgBuffer[(movingAvg.movingAvgIdx+(sizeOfMovingAverageBuffer-sizeOfDelay)) % sizeOfMovingAverageBuffer] > minReliableData){
			// Use a delayed value as the threshold (before it loses speed)
			movingAvg.speedBeforeGotBall = movingAvg.movingAvgBuffer[(movingAvg.movingAvgIdx+(sizeOfMovingAverageBuffer-sizeOfDelay)) % sizeOfMovingAverageBuffer];
		}
	}
	// check if all conditions are met, assume we have the ball if so
	if (speed_reducing && (dribbler_filtered_measured_speed > minReliableData) && (AvgCommandedSpeed > 0)){
		hasBall = true;
	}
	
	// Only say we lose the ball if the speed increases above the threshold value or if the dribbler turns off
	if (hasBall == true){
		if ((speed_increasing && (dribbler_filtered_measured_speed > (movingAvg.speedBeforeGotBall-5))) || AvgCommandedSpeed < 0.05){
			hasBall = false;
		}
	}
	return hasBall;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

/**
 * @brief Reads out the counts of the dribbler encoder
 */
static int16_t getEncoderData(){
	uint32_t value = __HAL_TIM_GET_COUNTER(ENC_DRIBBLER);
	return value;
}

/**
 * @brief Resets the dribbler encoder
 */
static void resetDribblerEncoders() {
	__HAL_TIM_SET_COUNTER(ENC_DRIBBLER, 0);
}

/**
 * @brief Calculates angular velocity in rad/s for the dribbler based on their encoder values
 * 
 * @todo This function requires to be called every 100 milliseconds, as dictated by the variable TIME_DIFF contained
 * within the variable ENCODERtoOMEGA. This can of course not always be perfectly guaranteed. Therefore, a timer should
 * be used to calculate the time difference between two calculations.
 */
static void computeDribblerSpeed(){
	int16_t encoder_value = getEncoderData();
	resetDribblerEncoders();
	
	// Convert encoder values to rad/s
	dribbler_measured_speed = DRIBBLER_ENCODER_TO_OMEGA * (float) fabs(encoder_value);
}

/**
 * @brief Moving average filter for the dribbler speeds
 */
float smoothen_dribblerSpeed(float speed){
    movingAvg.movingAvgBuffer[movingAvg.movingAvgIdx] = speed;
	movingAvg.movingAvgIdx = (movingAvg.movingAvgIdx+1) % sizeOfMovingAverageBuffer;

    return getAvgOfBuffer(movingAvg.movingAvgBuffer, sizeOfMovingAverageBuffer);
} 

/**
 * @brief Calculates the average of an array (buffer)
 */
float getAvgOfBuffer(float *buffer, int size){
	if (size == 0) return 0.0; // make sure we don't divide by zero
	
	float sum = 0.0;
	for (int i=0; i<size; i++){
        sum += buffer[i];
    }

	return sum/(float)size;
}