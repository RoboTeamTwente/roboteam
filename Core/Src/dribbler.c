
#include "../Inc/dribbler.h"

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void dribbler_Init(){
	start_PWM(PWM_Dribbler);
	dribbler_SetSpeed(0);
}

void dribbler_DeInit(){
	stop_PWM(PWM_Dribbler);
}

void dribbler_SetSpeed(int speed){
	if(speed > 100){
		speed = 100;
	}else if(speed < 0){
		speed = 0;
	}

	// The 12V and 24V boards require different calculations for the dribbler PWM
	if (MOTORS_50W) {
		set_PWM(PWM_Dribbler, speed / 100.0 * MAX_PWM);
	} else {
		set_PWM(PWM_Dribbler, (100 - speed) * (MAX_PWM / 100));
	}
}
