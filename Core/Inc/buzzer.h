/*
 * buzzer.h
 *
 *  Created on: Apr 27, 2019
 *      Author: simen
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void buzzer_Init();

void buzzer_DeInit();

void buzzer_SetPWM(int pwm);

#endif /* INC_BUZZER_H_ */
