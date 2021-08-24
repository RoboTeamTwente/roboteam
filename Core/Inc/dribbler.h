
/* Description: Makes the dribbler spin
 *
 * Instructions:
 * 1) set speed (1(off)-7(max))
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef DRIBBLER_DRIBBLER_H_
#define DRIBBLER_DRIBBLER_H_

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void dribbler_Init();

void dribbler_DeInit();

void dribbler_SetSpeed(int speed);

#endif /* DRIBBLER_DRIBBLER_H_ */
