
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

#include "../Util/control_util.h"
#include <stdbool.h>

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

// Calibrate the state yaw with the vision yaw
void yaw_Calibrate(float xsensYaw, float visionYaw, bool visionAvailable, float rateOfTurn);

// Getter for the yaw after calibration
float yaw_GetCalibratedYaw();

// Check if we have calibrated at least once, otherwise the yaw is probably way off
bool yaw_hasCalibratedOnce();

#endif /* YAWCALIBRATION_H_ */
