
/* Description: Makes the robot chip or kick
 *
 * Instructions:
 * 1) First the robot's needs to charge
 * 2) when that has happened the kick or chip command can be executed
 *
 * Extra functions:
 *
 * Notes:
 * As opposed to other functionalities, shoot uses a callback() instead of an update(). This is because shoot
 * only needs to update when kicking is needed, while for example the velocity control needs to ran constantly. It would be
 * very inefficient to update shoot every cycle. The timer for the callback is set internally. The time after which another
 * callback should be made differs per shootState. While charging and kicking, updating has to be done frequently and while the
 * robot is ready to kick, updating can be done less frequent.
*/

#ifndef __shoot_H
#define __shoot_H

#include "gpio_util.h"
#include "tim_util.h"
#include "control_util.h"
#include <stdbool.h>

///////////////////////////////////////////////////// STRUCTS

typedef enum{
	shoot_Ready,
	shoot_Charging,
	shoot_Shooting,
	shoot_Off
}shoot_states;

typedef enum{
	shoot_Kick,
	shoot_Chip
}shoot_types;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void shoot_Init();

void shoot_DeInit();

void shoot_Callback();

void shoot_Shoot(shoot_types type);

shoot_states shoot_GetState();

void shoot_SetPower(float input);

#endif /* __shoot_H */
