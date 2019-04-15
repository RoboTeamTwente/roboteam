
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

#ifndef WHEELS_H_
#define WHEELS_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

int wheels_Init();

int wheels_Deinit();

void wheels_Update();

void wheels_setRef(float input[4]);

float wheels_GetState(wheel_names wheel);

int wheels_GetPWM(wheel_names wheel);

#endif /* WHEELS_H_ */
