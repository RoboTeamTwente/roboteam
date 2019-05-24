#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// BUZZER LIMITERS
#define BUZZER_SCALER 4000
#define MAX_BUZZER_DURATION 1000000

///////////////////////////////////////////////////// TONES
// tones are counters, so (1e6/tone_freq - 1) in buzzer_Play()
// TO DO: remove unplayable tones
#define buzz_Silence 0
#define buzz_B0  31
#define buzz_C1  33
#define buzz_CS1 35
#define buzz_D1  37
#define buzz_DS1 39
#define buzz_E1  41
#define buzz_F1  44
#define buzz_FS1 46
#define buzz_G1  49
#define buzz_GS1 52
#define buzz_A1  55
#define buzz_AS1 58
#define buzz_B1  62
#define buzz_C2  65
#define buzz_CS2 69
#define buzz_D2  73
#define buzz_DS2 78
#define buzz_E2  82
#define buzz_F2  87
#define buzz_FS2 93
#define buzz_G2  98
#define buzz_GS2 104
#define buzz_A2  110
#define buzz_AS2 117
#define buzz_B2  123
#define buzz_C3  131
#define buzz_CS3 139
#define buzz_D3  147
#define buzz_DS3 156
#define buzz_E3  165
#define buzz_F3  175
#define buzz_FS3 185
#define buzz_G3  196
#define buzz_GS3 208
#define buzz_A3  220
#define buzz_AS3 233
#define buzz_B3  247
#define buzz_C4  262
#define buzz_CS4 277
#define buzz_D4  294
#define buzz_DS4 311
#define buzz_E4  330
#define buzz_F4  349
#define buzz_FS4 370
#define buzz_G4  392
#define buzz_GS4 415
#define buzz_A4  440
#define buzz_AS4 466
#define buzz_B4  494
#define buzz_C5  523
#define buzz_CS5 554
#define buzz_D5  587
#define buzz_DS5 622
#define buzz_E5  659
#define buzz_F5  698
#define buzz_FS5 740
#define buzz_G5  784
#define buzz_GS5 831
#define buzz_A5  880
#define buzz_AS5 932
#define buzz_B5  988
#define buzz_C6  1047
#define buzz_CS6 1109
#define buzz_D6  1175
#define buzz_DS6 1245
#define buzz_E6  1319
#define buzz_F6  1397
#define buzz_FS6 1480
#define buzz_G6  1568
#define buzz_GS6 1661
#define buzz_A6  1760
#define buzz_AS6 1865
#define buzz_B6  1976
#define buzz_C7  2093
#define buzz_CS7 2217
#define buzz_D7  2349
#define buzz_DS7 2489
#define buzz_E7  2637
#define buzz_F7  2794
#define buzz_FS7 2960
#define buzz_G7  3136
#define buzz_GS7 3322
#define buzz_A7  3520
#define buzz_AS7 3729
#define buzz_B7  3951
#define buzz_C8  4186
#define buzz_CS8 4435
#define buzz_D8  4699
#define buzz_DS8 4978

///////////////////////////////////////////////////// PUBLIC VARIABLE DECLARATIONS
typedef struct song_struct{
	uint16_t period;
	float duration;
} song_struct;


///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS
void buzzer_Init();

void buzzer_DeInit();

void buzzer_Callback();

void buzzer_SetPWM_Duty(uint16_t duty);

// add functions to play songs here, call in other files where needed they will use internal setpwm_period and play functions
void buzzer_Play_Startup();

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS
void buzzer_SetPWM_Period(uint16_t period);

void buzzer_Play(song_struct* tone);


#endif /* INC_BUZZER_H_ */
