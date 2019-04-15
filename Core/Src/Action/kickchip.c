
#include "../Inc/Action/kickchip.h"
#include <stdbool.h>

//TODO: Check if this works

///////////////////////////////////////////////////// STRUCTS

kick_states kickState = Off;

///////////////////////////////////////////////////// VARIABLES

static bool charged = false;
static int percentage = 0;

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void kick_Init(){
	charged = false;
	kickState = charge;
	set_Pin(Kick_pin,GPIO_PIN_RESET);		// Kick off
	set_Pin(Chip_pin, GPIO_PIN_RESET);		// Chip off
	set_Pin(Charge_pin, GPIO_PIN_SET);		// kick_Charging on
	HAL_TIM_Base_Start(ENC_KC);
}

void kick_DeInit(){
	kickState = Off;
	charged = false;
	set_Pin(Kick_pin, GPIO_PIN_RESET);		// Kick off
	set_Pin(Chip_pin, GPIO_PIN_RESET);		// Chip off
	set_Pin(Charge_pin, GPIO_PIN_RESET);	// kick_Charging off
	HAL_TIM_Base_Stop(ENC_KC);
}

//TODO: why does kickState behave so weird
void kick_Callback()
{
	switch(kickState){
	case On:
		break;
	case charge:
		if(read_Pin(Charge_done_pin)){
			set_Pin(Charge_pin, 0);
			charged = true;
			kickState = On;
		}
		else {
			set_Pin(Kick_pin, 0);
			set_Pin(Chip_pin, 0);
			set_Pin(Charge_pin, 1);
			charged = false;
		}
		break;
	case kick:
		if (charged){
			kick_Shoot();
		} else {
			kickState = charge;
		}
		break;
	case chip:
		if (charged){
			kick_Chip();
		} else {
			kickState = charge;
		}
		break;
	case Off:
		break;
	}
}


void kick_Shoot()
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(charged) {
		set_Pin(Kick_pin, 1);
	} else {
		kickState = charge;
		charged = false;
	}
}

void kick_Chip()
{
	if(percentage < 1){
		percentage = 1;
	}else if(percentage > 100){
		percentage = 100;
	}
	if(charged) {
		set_Pin(Chip_pin, 1);
	} else {
		kickState = charge;
		charged = false;
	}
}

void kick_SetPer(int input){
	percentage = input;
}

