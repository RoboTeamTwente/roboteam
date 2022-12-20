
/* Description: Calibrates the Yaw
 *
 * Instructions:
 * 1) compares the xsens yaw + offset with the vision yaw
 * 2) If the difference is bigger than a certain threshold change the offset
 * 3) when vision is not updated, use the Gyro to derive the expected vision yaw
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef YAWCALIBRATION_H_
#define YAWCALIBRATION_H_

#include "control_util.h"
#include <stdbool.h>

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

/**
 * Calibrates the yaw measured by the IMU.
 * 
 * @param xsensYaw          The yaw as currently being measured by the IMU.
 * @param visionYaw         The yaw as currently being observed by vision
 * @param visionAvailable   Wether vision can be used at this moment
 * @param rateOfTurn        Our current rate of turn (if it is too high, we cannot reliably calibrate the yaw)
 */
void yaw_Calibrate(float xsensYaw, float visionYaw, bool visionAvailable, float rateOfTurn);

/**
 * Get the calibrated yaw
 * 
 * @return float 
 */
float yaw_GetCalibratedYaw();

/**
 * Check if the calibration has been executed at least once.
 * Otherwise the yaw is probably way off
 * 
 * @return bool
 */
bool yaw_hasCalibratedOnce();

/**
 * Resets the entire yaw calibration on the next calibration call
 */
void yaw_ResetCalibration();

#endif /* YAWCALIBRATION_H_ */
