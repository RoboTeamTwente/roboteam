/*
 * yawCalibration.h
 *
 *  Created on: Jan 8, 2019
 *      Author: simen
 */

#ifndef YAWCALIBRATION_H_
#define YAWCALIBRATION_H_

#include "stdbool.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void yaw_Calibrate(float xsensYaw, float visionYaw);
float yaw_GetCalibratedYaw();

#endif /* YAWCALIBRATION_H_ */
