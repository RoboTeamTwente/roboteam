
#include "yawCalibration.h"
#include "PuTTY.h"

///////////////////////////////////////////////////// DEFINITIONS

#define BUFFER_SIZE 5 							// assume 50 ms (5 time steps) delay between vision and XSens
#define CALIBRATION_TIME 0.2f 					// number of seconds to do for averaging
#define MAX_RATE_OF_TURN (M_PI/2.0f) / 4.0f 	// highest rate of turn (rad/s) allowed to do calibration

///////////////////////////////////////////////////// VARIABLES

static float calibratedYaw = 0.0f;
static bool hasCalibratedOnce = false;

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// If the vision yaw and xsens yaw deviate too much for several time steps, set calibration needed to true
static bool isCalibrationNeeded(float visionYaw, float xsensYaw, float yawOffset);

// Check if robot has been rotating sufficiently slow for several time steps
static bool isRotatingSlow(float visionYaw);

// Get the oldest Xsens yaw from the buffer (as old as the assumed delay)
static float getOldXsensYaw(float newXsensYaw);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void yaw_Calibrate(float newXsensYaw, float visionYaw, bool visionAvailable, float rateOfTurn) {
	static float yawOffset = 0.0f;
	static int restCounter = 0;
	static float sumXsensVec[2] = {0.0f};
	static float sumVisionVec[2] = {0.0f};

	// Calibrate the xsens yaw from some time ago with vision to account for delay while sending.
	float oldXsensYaw = getOldXsensYaw(newXsensYaw);

	if (isCalibrationNeeded(visionYaw, oldXsensYaw, yawOffset) && isRotatingSlow(rateOfTurn) && visionAvailable) {
		if (restCounter > CALIBRATION_TIME / TIME_DIFF) {
			// calculate offset
			float avgVisionYaw = atan2f(sumVisionVec[1], sumVisionVec[0]);
			float avgXsensYaw = atan2f(sumXsensVec[1], sumXsensVec[0]);
			yawOffset = constrainAngle(avgVisionYaw - avgXsensYaw);
			hasCalibratedOnce = true;
			restCounter = 0;
		} else {
			// Sum the unit vectors with these angles and then take the angle of the resulting vector.
			sumXsensVec[0] += cosf(oldXsensYaw);
			sumXsensVec[1] += sinf(oldXsensYaw);
			sumVisionVec[0] += cosf(visionYaw);
			sumVisionVec[1] += sinf(visionYaw);
			restCounter++;
		}
	} else {
		restCounter = 0;
		sumXsensVec[0] = 0.0f;
		sumXsensVec[1] = 0.0f;
		sumVisionVec[0] = 0.0f;
		sumVisionVec[1] = 0.0f;
	}

	// Add the calculated offset to the newly measured xsens yaw.
	calibratedYaw = constrainAngle(newXsensYaw + yawOffset);

}

float yaw_GetCalibratedYaw(){
	return calibratedYaw;
}

bool yaw_hasCalibratedOnce(){
	return hasCalibratedOnce;
};

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static bool isCalibrationNeeded(float visionYaw, float xsensYaw, float yawOffset) {

	static bool calibrationNeeded = false;
	static int checkCounter = 0;
	if (fabs(constrainAngle(visionYaw - (xsensYaw + yawOffset))) > M_PI/180) { // require 1 degree accuracy
		checkCounter++;
	} else {
		checkCounter = 0;
		calibrationNeeded = false;
	}
	if (checkCounter > 10) {
		checkCounter = 0;
		calibrationNeeded = true;
	}
	return calibrationNeeded;
}

static bool isRotatingSlow(float rateOfTurn) {
	return fabs(rateOfTurn) < MAX_RATE_OF_TURN;
}

static float getOldXsensYaw(float newXsensYaw) {
	static float buffer[BUFFER_SIZE] = {0.0f};
	static int index = 0;

	float oldXsensYaw = buffer[index];
	buffer[index] = newXsensYaw;
	index++;
	index = (index >= BUFFER_SIZE) ? 0 : index;
	return oldXsensYaw;
}

