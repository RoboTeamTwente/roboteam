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

///////////////////////////////////////////////////// DEFINITIONS

#define BUFFER_SIZE 5 // assume 50 ms (5 time steps) delay between vision and XSens
#define restDuration 20 // number of time steps to do for averaging TODO: test this

///////////////////////////////////////////////////// VARIABLES

static float calibratedYaw = 0.0f;

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//If the vision yaw and xsens yaw deviate too much for several time steps, set calibration needed to true
static bool isCalibrationNeeded(float visionYaw, float xsensYaw, float yawOffset);

//Check if robot has been rotating sufficiently slow for several time steps
static bool isRotatingSlow(float visionYaw);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void yaw_Calibrate(float xsensYaw, float visionYaw, bool visionAvailable) {
	static float yawOffset = 0.0f;
	static int restCounter = 0;
	static float sumXsensVec[2] = {0.0f};
	static float sumVisionVec[2] = {0.0f};
	static float prevVisionYaw = 0.0f;

	if (visionYaw == prevVisionYaw) {
		visionYaw += MT_GetGyro()[2] * TIME_DIFF;
	}

	if (isCalibrationNeeded(visionYaw, xsensYaw, yawOffset) && isRotatingSlow(visionYaw) && visionAvailable) {
		if (restCounter > restDuration) {
			// calculate offset
			float avgVisionYaw = atan2f(sumVisionVec[1], sumVisionVec[0]);
			float avgXsensYaw = atan2f(sumXsensVec[1], sumXsensVec[0]);
			yawOffset = constrainAngle(avgVisionYaw - avgXsensYaw);
			restCounter = 0;
		} else {
			// Sum the unit vectors with these angles and then take the angle of the resulting vector.
			sumXsensVec[0] += cosf(xsensYaw);
			sumXsensVec[1] += sinf(xsensYaw);
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

	bufferYaw(xsensYaw);
	prevVisionYaw = visionYaw;
	calibratedYaw = constrainAngle(xsensYaw + yawOffset);
}

float yaw_GetCalibratedYaw(){
	return calibratedYaw;
}

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


static bool isRotatingSlow(float visionYaw) {
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

