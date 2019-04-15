
/* Description:
 *
 * Instructions:
 * 1)
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
