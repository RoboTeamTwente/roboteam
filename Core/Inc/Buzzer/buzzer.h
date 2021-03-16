#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"
#include "buzzer_Tunes.h"

///////////////////////////////////////////////////// BUZZER LIMITERS
#define BUZZER_SCALER 4000
#define MAX_BUZZER_DURATION 1000000

///////////////////////////////////////////////////// PUBLIC VARIABLE DECLARATIONS
extern uint32_t buzzer_Duration;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS
void buzzer_Init();

void buzzer_DeInit();

void buzzer_Callback();

void buzzer_SetPWM_Duty(uint16_t duty);

// add functions to play songs here, call in other files where needed
void buzzer_Play_QuickBeepUp();
void buzzer_Play_QuickBeepDown();
void buzzer_Play_Startup();
void buzzer_Play_Tetris();
void buzzer_Play_Mario();
void buzzer_Play_PowerUp();
void buzzer_Play_WarningOne();
void buzzer_Play_WarningTwo();
void buzzer_Play_WarningThree();
void buzzer_Play_WarningFour();
void buzzer_Play_BridgeBattle();
void buzzer_Play_ImperialMarch();
void buzzer_Play_Flatline();
void buzzer_Play_HBD();

void buzzer_Play_ID(uint8_t id);
///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
void buzzer_SetPWM_Period(uint16_t period);
void buzzer_Play(song_struct* tone);


#endif /* INC_BUZZER_H_ */
