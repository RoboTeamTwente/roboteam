
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

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void yaw_Calibrate(float xsensYaw, float visionYaw, bool visionAvailable);

float yaw_GetCalibratedYaw();

#endif /* YAWCALIBRATION_H_ */
