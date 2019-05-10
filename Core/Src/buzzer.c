/*
 * buzzer.c
 *
 *  Created on: Apr 27, 2019
 *      Author: simen
 */
#include "../Inc/buzzer.h"

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

//TODO: make it work, low priority

void buzzer_Init() {
	start_PWM(PWM_Buzzer);
	buzzer_SetPWM(0);
}

void buzzer_DeInit() {
	stop_PWM(PWM_Buzzer);
}

void buzzer_SetPWM(int pwm) {
	if (pwm > MAX_PWM) {
		pwm = MAX_PWM;
	} else if (pwm < 0) {
		pwm = 0;
	}
	set_PWM(PWM_Buzzer, pwm);
}
