
#include "../Inc/dribbler.h"

///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void dribbler_Init(){
	start_PWM(PWM_Dribbler);
	dribbler_SetSpeed(0);
}

void dribbler_Deinit(){
	stop_PWM(PWM_Dribbler);
}

void dribbler_SetSpeed(int speed){
	if(speed > 100){
		speed = 100;
	}else if(speed < 0){
		speed = 0;
	}
	set_PWM(PWM_Dribbler, (100 - speed) * (MAX_PWM / 100));
}
