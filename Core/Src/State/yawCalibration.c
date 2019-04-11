/*
 * yawCalibration.c
 *
 *  Created on: Jan 8, 2019
 *      Author: simen
 */

#include "yawCalibration.h"
#include "stdbool.h"
#include <math.h>
#include "../Util/control_util.h"

//------------ settings for testing ----------------------
bool alwaysCalibrate = false;
bool compensateWithGyro = true;
//--------------------------------------------------------

///////////////////////////////////////////////////// DEFINITIONS
#define BUFFER_SIZE 5 // assume 50 ms (5 time steps) delay between vision and XSens
#define restDuration 20 // number of time steps to do for averaging TODO: test this

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
bool isCalibrationNeeded(float visionYaw, float xsensYaw, float yawOffset);
bool isRotatingSlow(float visionYaw);
void bufferYaw(float xsensYaw);
void rotate(float yaw, float input[3], float output[3]);

static float xsensYawBuffer[BUFFER_SIZE] = {0, 0, 0, 0, 0};
static int bufferIndex = 0;

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS
void calibrateXsens(float xsensData[3], float visionYaw, bool visionAvailable) {
	static float yawOffset = 0;
	static int restCounter = 0;
	static float avgXsensVec[2] = {0}, avgVisionVec[2] = {0};
	static float prevVisionYaw = 0;
	float xsensYaw = xsensData[body_w];

	if (compensateWithGyro && visionYaw == prevVisionYaw) {
		visionYaw += MT_GetGyro()[2] * TIME_DIFF;
	}

	if (isCalibrationNeeded(visionYaw, xsensYaw, yawOffset) && isRotatingSlow(visionYaw) && visionAvailable) {
		if (restCounter > restDuration) {
			// calculate offset
			float avgVisionYaw = atan2f(avgVisionVec[1], avgVisionVec[0]);
			float avgXsensYaw = atan2f(avgXsensVec[1], avgXsensVec[0]);
			yawOffset = constrainAngle(avgVisionYaw - avgXsensYaw);
			restCounter = 0;
		} else {
			// Taking the average of the angles makes no sense, since Pi and -Pi would not be the same angle.
			// Instead, sum the unit vectors with these angles and then take the angle of the resulting vector.
			avgXsensVec[0] += cosf(xsensYaw);
			avgXsensVec[1] += sinf(xsensYaw);
			avgVisionVec[0] += cosf(visionYaw);
			avgVisionVec[1] += sinf(visionYaw);
			restCounter++;
		}
	} else {
		restCounter = 0;
		avgXsensVec[0] = 0; avgXsensVec[1] = 0;
		avgVisionVec[0] = 0; avgVisionVec[1] = 0;
	}
	bufferYaw(xsensYaw);
	prevVisionYaw = visionYaw;

	rotate(yawOffset, xsensData, xsensData);
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS
bool isCalibrationNeeded(float visionYaw, float xsensYaw, float yawOffset) {
	// if vision yaw and xsens yaw deviate too much for several time steps, set calibration needed to true

	//--------- testing ---------
	if (alwaysCalibrate) {
		return true;
	}
	xsensYaw = compensateWithGyro ? xsensYaw : xsensYawBuffer[bufferIndex];
	//---------------------------

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

bool isRotatingSlow(float visionYaw) {
	// Check if robot has been rotating sufficiently slow for several time steps
	static bool rotatingSlow = false;
	static int rotateCounter = 0;
	static float startYaw = 0;
	if (fabs(constrainAngle(startYaw - visionYaw)) < 0.01) {
		rotateCounter++;
	} else {
		rotateCounter = 0;
		startYaw = visionYaw;
		rotatingSlow = false;
	}
	if (rotateCounter > 10) {
		rotateCounter = 0;
		startYaw = visionYaw;
		rotatingSlow = true;
	}
	return rotatingSlow;
}

void bufferYaw(float xsensYaw) {
	xsensYawBuffer[bufferIndex] = xsensYaw;
	bufferIndex = bufferIndex >= BUFFER_SIZE ? 0 : bufferIndex + 1;
}

void rotate(float yawOffset, float input[3], float output[3]){
	float outX = cosf(yawOffset)*input[body_x] - sinf(yawOffset)*input[body_y];
	float outY = sinf(yawOffset)*input[body_x] + cosf(yawOffset)*input[body_y];

	output[body_x] = outX;
	output[body_y] = outY;
	output[body_w] = constrainAngle(input[body_w] + yawOffset);
}

