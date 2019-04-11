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
void calibrateXsens(float xsensData[3], float visionYaw);

#endif /* YAWCALIBRATION_H_ */
