
#include <shoot.h>
#include <BaseTypes.h>

///////////////////////////////////////////////////// STRUCTS

static shoot_states shootState = shoot_Off;

///////////////////////////////////////////////////// VARIABLES

static bool charged = false;	// true if the capacitor is fully charged
static float power = 0; 		// percentage of maximum shooting power [0,6.5]

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

// Stops and starts the timer for a certain period of time
void resetTimer(int timePeriod);

int calculateShootingTime(shoot_types type);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void shoot_Init(){
	charged = false;
	shootState = shoot_Charging;
	set_Pin(Kick_pin,0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 1);		// shoot_Charging on
	shoot_Callback(); 			// go to callback for the first time
}

void shoot_DeInit(){
	charged = false;
	shootState = shoot_Off;
	set_Pin(Kick_pin, 0);		// Kick off
	set_Pin(Chip_pin, 0);		// Chip off
	set_Pin(Charge_pin, 0);		// shoot_Charging off
	HAL_TIM_Base_Stop(TIM_SHOOT);
}

void shoot_Callback()
{
	int callbackTime = 0;
	static int count = 0;

	switch(shootState){
	case shoot_Ready:
		set_Pin(Charge_pin, 1); // Keep charging
		callbackTime = TIMER_FREQ/READY_CALLBACK_FREQ;
		break;
	case shoot_Charging:
		if (count >= 5) {
			charged = true;
			count = 0;
			shootState = shoot_Ready;
		}
		else {
			set_Pin(Kick_pin, 0);
			set_Pin(Chip_pin, 0);
			set_Pin(Charge_pin, 1);
			charged = false;
			count++;
		}
		callbackTime = TIMER_FREQ/CHARGING_CALLBACK_FREQ;
		break;
	case shoot_Shooting:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		shootState = shoot_Charging;
		callbackTime = TIMER_FREQ/SHOOTING_CALLBACK_FREQ;
		break;
	case shoot_Off:
		set_Pin(Kick_pin, 0);		// Kick off
		set_Pin(Chip_pin, 0);		// Chip off
		set_Pin(Charge_pin, 0);		// kick_Charging off
		callbackTime = TIMER_FREQ/OFF_CALLBACK_FREQ;
		break;
	}
	resetTimer(callbackTime);
}

shoot_states shoot_GetState(){
	return shootState;
}

void shoot_SetPower(float meters_per_second){
    // At some point, make a formula to convert m/s to power. For now, linear relation
    power = meters_per_second / PACKET_RANGE_ROBOT_COMMAND_KICK_CHIP_POWER_MAX;
    if(power < 0) power = 0;
    if(1 < power) power = 1;
}

void shoot_Shoot(shoot_types type)
{
	if(shootState == shoot_Ready)
	{
//		Putty_printf("shooting! power = %d \n\r", power);
		shootState = shoot_Shooting;
		set_Pin(Charge_pin, 0); 									// Disable shoot_Charging
		set_Pin(type == shoot_Kick ? Kick_pin : Chip_pin, 1); 		// Kick/Chip on

		resetTimer(calculateShootingTime(type));
	}
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void resetTimer(int timePeriod)
{
	HAL_TIM_Base_Stop(TIM_SHOOT);							// Stop timer
	__HAL_TIM_CLEAR_IT(TIM_SHOOT, TIM_IT_UPDATE);			// Clear timer
	__HAL_TIM_SET_COUNTER(TIM_SHOOT, 0);      				// Reset timer
	__HAL_TIM_SET_AUTORELOAD(TIM_SHOOT, timePeriod);		// Set callback time to defined value
	HAL_TIM_Base_Start_IT(TIM_SHOOT);						// Start timer
}

int calculateShootingTime(shoot_types type) {
	if (type == shoot_Kick) {
		int kickTime = MIN_KICK_TIME + power * (MAX_KICK_TIME-MIN_KICK_TIME);
		if(kickTime < MIN_KICK_TIME) kickTime = MIN_KICK_TIME;
		if(MAX_KICK_TIME < kickTime) kickTime = MAX_KICK_TIME;
		return kickTime;

	} else if (type == shoot_Chip) {
		int chipTime = MIN_CHIP_TIME + power * (MAX_KICK_TIME-MIN_KICK_TIME);
		if(chipTime < MIN_CHIP_TIME) chipTime = MIN_CHIP_TIME;
		if(MAX_CHIP_TIME < chipTime) chipTime = MAX_CHIP_TIME;
		return chipTime;
	}
	return 0;
}
