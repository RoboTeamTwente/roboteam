#include "../Inc/buzzer.h"
#include "../Util/tim_util.h"
#include "buzzer_Tunes.h"

uint32_t buzzer_Duration = 0;

song_struct* song;

song_struct startup_song[] = {{buzz_C4, 0.1}, {buzz_D4, 0.1}, {buzz_E4, 0.1}, {buzz_F4, 0.1}, {buzz_G4, 0.1}, {buzz_A4, 0.1}, {buzz_B4, 0.1}, {buzz_C5, 0.1}, {0xFFFF, 0}};
song_struct tetris[] = {{buzz_E5, Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_A4, Tbeat}, {buzz_A4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_Silence, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_E5, Tbeat}, {buzz_C5, Tbeat}, {buzz_A4, Tbeat}, {buzz_A4, Tbeat}, {buzz_Silence, 1.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_G5, 0.5*Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_Silence, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat},{buzz_E5,Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_A4,Tbeat}, {buzz_A4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5,Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_Silence, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat}, {buzz_Silence,1.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_G5, 0.5*Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_Silence, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5,Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat}, {buzz_E4,2*Tbeat}, {buzz_C4,2*Tbeat}, {buzz_D4,2*Tbeat}, {buzz_B3,2*Tbeat}, {buzz_C4,2*Tbeat}, {buzz_A3,2*Tbeat}, {buzz_GS3,2*Tbeat}, {buzz_B3,Tbeat}, {buzz_Silence,Tbeat}, {buzz_E4,2*Tbeat}, {buzz_C4,2*Tbeat}, {buzz_D4,2*Tbeat}, {buzz_B3,2*Tbeat}, {buzz_C4,Tbeat}, {buzz_E4,Tbeat}, {buzz_A4,2*Tbeat}, {buzz_GS4,3*Tbeat}, {buzz_Silence,Tbeat}, {0xFFFF, 0}};
song_struct mario_victory[] = {{buzz_GS3,Mbeat/3.0}, {buzz_CS4,Mbeat/3.0}, {buzz_F4,Mbeat/3.0}, {buzz_GS4,Mbeat/3.0}, {buzz_CS5,Mbeat/3.0}, {buzz_F5,Mbeat/3.0}, {buzz_GS5,Mbeat}, {buzz_F5,Mbeat}, {buzz_A3,Mbeat/3.0}, {buzz_CS4,Mbeat/3.0}, {buzz_E4,Mbeat/3.0}, {buzz_A4,Mbeat/3.0}, {buzz_CS5,Mbeat/3.0}, {buzz_E5,Mbeat/3.0}, {buzz_A5,Mbeat}, {buzz_E5,Mbeat}, {buzz_B3,Mbeat/3.0}, {buzz_DS4,Mbeat/3.0}, {buzz_FS4,Mbeat/3.0}, {buzz_B4,Mbeat/3.0}, {buzz_DS5,Mbeat/3.0}, {buzz_FS5,Mbeat/3.0}, {buzz_B5,Mbeat}, {buzz_B5,Mbeat/3.0}, {buzz_B5,Mbeat/3.0}, {buzz_B5,Mbeat/3.0}, {buzz_CS6,Mbeat*2.0},{0xFFFF, 0}};
song_struct powerUp[] = {{buzz_C5,fBeat*0.5},{buzz_G4,fBeat},{buzz_C5,fBeat},{buzz_E5,fBeat},{buzz_G5,fBeat},{buzz_C6,fBeat},{buzz_G5,fBeat},{buzz_C5,fBeat*0.5},{buzz_GS4,fBeat},{buzz_C5,fBeat},{buzz_DS5,fBeat},{buzz_GS5,fBeat},{buzz_DS5,fBeat},{buzz_GS5,fBeat},{buzz_C6,fBeat},{buzz_DS6,fBeat},{buzz_GS6,fBeat},{buzz_DS6,fBeat},{buzz_D5,0.5*fBeat},{buzz_AS4,fBeat},{buzz_D5,fBeat},{buzz_F5,fBeat},{buzz_AS5,fBeat},{buzz_F5,fBeat},{buzz_AS5,fBeat},{buzz_D6,fBeat},{buzz_F6,fBeat},{buzz_AS6,fBeat},{buzz_F6,fBeat},{0xFFFF, 0}};
///////////////////////////////////////////////////// PUBLIC FUNCTIONS IMPLEMENTATIONS

void buzzer_Init() {
	HAL_TIM_Base_Stop(PWM_Buzzer.TIM);
	buzzer_Duration = 0;
	// play a sound to inform that we are aliiiiiiiiive
	buzzer_Play_Startup();
}

void buzzer_DeInit() {
	HAL_TIM_PWM_Stop(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
	HAL_TIM_Base_Stop_IT(PWM_Buzzer.TIM);
}

void buzzer_Callback() {
	if(song){
		if(buzzer_Duration != 0){
			buzzer_Duration--;
		} else {
			song++;
			HAL_TIM_PWM_Stop(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
			HAL_TIM_Base_Stop_IT(PWM_Buzzer.TIM);
			if(song->period == 0xFFFF){
				song = NULL;
				return;
			}
			buzzer_Play(song);
		}
	}
}

void buzzer_SetPWM_Duty(uint16_t duty) {
	set_PWM(PWM_Buzzer, duty);
}

void buzzer_SetPWM_Period(uint16_t period) {
	// HAL library macro to set timer PWM period in runtime
	__HAL_TIM_SET_AUTORELOAD(PWM_Buzzer.TIM, period);
}

void buzzer_Play(song_struct* tone) {
	tone->period = (tone->period == 0) ? 0xFFFF : ((1e6 / tone->period)-1);
	buzzer_Duration = (1e6 / tone->period) * (tone->duration);

	if (tone->period == 0xFFFF) {
		HAL_TIM_Base_Start_IT(PWM_Buzzer.TIM);
		buzzer_SetPWM_Period(tone->period);
		buzzer_SetPWM_Duty(tone->period/2);
	}
	else {
		// set the period and duty cycle
		buzzer_SetPWM_Period(tone->period);
		buzzer_SetPWM_Duty(tone->period/2);
		HAL_TIM_Base_Start_IT(PWM_Buzzer.TIM);
		HAL_TIM_PWM_Start(PWM_Buzzer.TIM, PWM_Buzzer.Channel);
	}
}

void buzzer_Play_Startup(){
	song = startup_song;
	buzzer_Play(song);
}

void buzzer_Play_Tetris(){
	song = tetris;
	buzzer_Play(song);
}

void buzzer_Play_Mario(){
	song = mario_victory;
	buzzer_Play(song);
}
void buzzer_Play_PowerUp(){
	song = powerUp;
	buzzer_Play(song);
}
