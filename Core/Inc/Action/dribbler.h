
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

#ifndef DRIBBLER_DRIBBLER_H_
#define DRIBBLER_DRIBBLER_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void dribbler_Init();

void dribbler_Deinit();

void dribbler_SetSpeed(int speed);

#endif /* DRIBBLER_DRIBBLER_H_ */
