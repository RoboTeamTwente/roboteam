/*
 * buzzer_Tunes.h
 *
 *  Created on: May 25, 2019
 *      Author: 454b
 */

#ifndef BUZZER_TUNES_H_
#define BUZZER_TUNES_H_

// BPM definitions
#define Tbeat 0.4
#define Mbeat 0.4
#define fBeat 0.0336
#define halfBeat 0.168

///////////////////////////////////////////////////// TONES
// tones are counters, so (1e6/tone_freq - 1) in buzzer_Play()
#define buzz_Si 0
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

// tunes
song_struct startup_song[] =
						{{buzz_C4, 0.1}, {buzz_D4, 0.1}, {buzz_E4, 0.1}, {buzz_F4, 0.1}, {buzz_G4, 0.1}, {buzz_A4, 0.1}, {buzz_B4, 0.1}, {buzz_C5, 0.1}, {0xFFFF, 0}};

song_struct tetris[] =
						{{buzz_E5, Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_A4, Tbeat},
						{buzz_A4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_Si, 0.5*Tbeat},
						{buzz_C5, 0.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_E5, Tbeat}, {buzz_C5, Tbeat}, {buzz_A4, Tbeat}, {buzz_A4, Tbeat}, {buzz_Si, 1.5*Tbeat}, {buzz_D5, Tbeat},
						{buzz_F5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_G5, 0.5*Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_Si, 0.5*Tbeat},
						{buzz_C5, 0.5*Tbeat}, {buzz_E5, Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat},
						{buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat}, {buzz_E5,Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat},
						{buzz_C5, 0.5*Tbeat}, {buzz_B4, 0.5*Tbeat}, {buzz_A4,Tbeat}, {buzz_A4, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5,Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat},
						{buzz_B4, Tbeat}, {buzz_Si, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat},
						{buzz_Si,1.5*Tbeat}, {buzz_D5, Tbeat}, {buzz_F5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_A5, 0.5*Tbeat}, {buzz_G5, 0.5*Tbeat}, {buzz_F5, 0.5*Tbeat},
						{buzz_E5, Tbeat}, {buzz_Si, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_E5,Tbeat}, {buzz_D5, 0.5*Tbeat}, {buzz_C5, 0.5*Tbeat}, {buzz_B4, Tbeat}, {buzz_B4, 0.5*Tbeat},
						{buzz_C5, 0.5*Tbeat}, {buzz_D5,Tbeat}, {buzz_E5,Tbeat}, {buzz_C5,Tbeat}, {buzz_A4,Tbeat}, {buzz_A4,Tbeat}, {buzz_E4,2*Tbeat}, {buzz_C4,2*Tbeat},
						{buzz_D4,2*Tbeat}, {buzz_B3,2*Tbeat}, {buzz_C4,2*Tbeat}, {buzz_A3,2*Tbeat}, {buzz_GS3,2*Tbeat}, {buzz_B3,Tbeat}, {buzz_Si,Tbeat}, {buzz_E4,2*Tbeat},
						{buzz_C4,2*Tbeat}, {buzz_D4,2*Tbeat}, {buzz_B3,2*Tbeat}, {buzz_C4,Tbeat}, {buzz_E4,Tbeat}, {buzz_A4,2*Tbeat}, {buzz_GS4,3*Tbeat}, {buzz_Si,Tbeat}, {0xFFFF, 0}};

song_struct mario_victory[] =
						{{buzz_GS3,Mbeat/3.0}, {buzz_CS4,Mbeat/3.0}, {buzz_F4,Mbeat/3.0}, {buzz_GS4,Mbeat/3.0}, {buzz_CS5,Mbeat/3.0}, {buzz_F5,Mbeat/3.0}, {buzz_GS5,Mbeat},
						{buzz_F5,Mbeat}, {buzz_A3,Mbeat/3.0}, {buzz_CS4,Mbeat/3.0}, {buzz_E4,Mbeat/3.0}, {buzz_A4,Mbeat/3.0}, {buzz_CS5,Mbeat/3.0}, {buzz_E5,Mbeat/3.0},
						{buzz_A5,Mbeat}, {buzz_E5,Mbeat}, {buzz_B3,Mbeat/3.0}, {buzz_DS4,Mbeat/3.0}, {buzz_FS4,Mbeat/3.0}, {buzz_B4,Mbeat/3.0}, {buzz_DS5,Mbeat/3.0},
						{buzz_FS5,Mbeat/3.0}, {buzz_B5,Mbeat}, {buzz_B5,Mbeat/3.0}, {buzz_B5,Mbeat/3.0}, {buzz_B5,Mbeat/3.0}, {buzz_CS6,Mbeat*2.0}, {0xFFFF, 0}};

song_struct powerUp[] =
						{{buzz_C5,fBeat*0.5}, {buzz_G4,fBeat}, {buzz_C5,fBeat}, {buzz_E5,fBeat}, {buzz_G5,fBeat}, {buzz_C6,fBeat}, {buzz_G5,fBeat}, {buzz_C5,fBeat*0.5}, {buzz_GS4,fBeat},
						{buzz_C5,fBeat}, {buzz_DS5,fBeat}, {buzz_GS5,fBeat}, {buzz_DS5,fBeat}, {buzz_GS5,fBeat}, {buzz_C6,fBeat}, {buzz_DS6,fBeat},
						{buzz_GS6,fBeat}, {buzz_DS6,fBeat}, {buzz_D5,0.5*fBeat}, {buzz_AS4,fBeat}, {buzz_D5,fBeat}, {buzz_F5,fBeat}, {buzz_AS5,fBeat},
						{buzz_F5,fBeat}, {buzz_AS5,fBeat}, {buzz_D6,fBeat}, {buzz_F6,fBeat}, {buzz_AS6,fBeat}, {buzz_F6,fBeat}, {0xFFFF, 0}};

song_struct warningOne[] =
						{{buzz_A5,0.1}, {buzz_E5,0.3}, {buzz_Si,0.2}, {buzz_A5,0.1}, {buzz_E5,0.3}, {buzz_Si,0.2}, {buzz_A5,0.1}, {buzz_E5,0.3}, {buzz_Si,0.2}, {0xFFFF, 0}};

song_struct warningTwo[] =
						{{buzz_C5,0.1}, {buzz_FS5,0.1}, {buzz_B5,0.6}, {buzz_C5,0.1}, {buzz_FS5,0.1}, {buzz_B5,0.1}, {buzz_Si,0.5}, {buzz_C5,0.1}, {buzz_FS5,0.1}, {buzz_B5,0.1},
						{buzz_Si,0.5}, {buzz_C5,0.1}, {buzz_FS5,0.1}, {buzz_B5,0.1}, {buzz_Si,0.5}, {0xFFFF, 0}};


song_struct warningThree[] =
						{{buzz_FS6,0.66}, {buzz_D6,0.66}, {buzz_B5,0.66}, {buzz_G5,0.66}, {buzz_AS5,1.33}, {buzz_Si,1.76}, {buzz_FS6,0.66}, {buzz_D6,0.66},
						{buzz_B5,0.66}, {buzz_G5,0.66}, {buzz_AS5,1.33}, {buzz_Si,1.76}, {buzz_FS6,0.66}, {buzz_D6,0.66}, {buzz_B5,0.66}, {buzz_G5,0.66}, {buzz_AS5,1.33},
						{buzz_Si,1.76}, {0xFFFF, 0}};

song_struct warningFour[] =
						{{buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_Si,0.2}, {buzz_G5,0.05}, {buzz_GS5,0.05},
						{buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_Si,0.2}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05},
						{buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_Si,0.2}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05},
						{buzz_Si,0.2}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_G5,0.05}, {buzz_GS5,0.05}, {buzz_Si,0.2}, {0xFFFF, 0}};

song_struct bridgeBattle[] =
						{{buzz_F4,halfBeat}, {buzz_F4,halfBeat}, {buzz_Si,halfBeat}, {buzz_F4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_Si,halfBeat},
						{buzz_F4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_B4,halfBeat}, {buzz_AS4,0.6666*halfBeat}, {buzz_B4,0.6666*halfBeat},
						{buzz_AS4,0.6666*halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_DS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_F4,halfBeat}, {buzz_Si,halfBeat},
						{buzz_F4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_G4,halfBeat}, {buzz_F4,halfBeat}, {buzz_Si,9*halfBeat}, {buzz_F4,halfBeat}, {buzz_G4,halfBeat},
						{buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_G4,halfBeat}, {buzz_F4,halfBeat}, {buzz_C5,halfBeat},
						{buzz_F4,halfBeat}, {buzz_CS5,halfBeat}, {buzz_F4,halfBeat}, {buzz_C5,halfBeat}, {buzz_GS4,2*halfBeat}, {buzz_GS4,halfBeat}, {buzz_AS4,halfBeat},
						{buzz_E4,halfBeat}, {buzz_C5,halfBeat}, {buzz_E4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_G4,2*halfBeat}, {buzz_G4,halfBeat}, {buzz_GS4,halfBeat},
						{buzz_G4,halfBeat}, {buzz_F4,halfBeat}, {buzz_G4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_G4,halfBeat},
						{buzz_F4,halfBeat}, {buzz_G4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_G4,halfBeat},
						{buzz_F4,halfBeat}, {buzz_C5,halfBeat}, {buzz_F4,halfBeat}, {buzz_CS5,halfBeat}, {buzz_F4,halfBeat}, {buzz_C5,halfBeat}, {buzz_GS4,2*halfBeat},
						{buzz_GS4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_E4,halfBeat}, {buzz_C5,halfBeat}, {buzz_E4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_G4,2*halfBeat},
						{buzz_G4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_C5,halfBeat}, {buzz_CS5,halfBeat}, {buzz_DS5,halfBeat}, {buzz_CS5,halfBeat},
						{buzz_C5,halfBeat}, {buzz_CS5,halfBeat}, {buzz_DS5,10*halfBeat}, {buzz_C5,halfBeat}, {buzz_CS5,halfBeat}, {buzz_DS5,2*halfBeat}, {buzz_G5,2*halfBeat},
						{buzz_F5,14*halfBeat}, {buzz_Si,2*halfBeat}, {buzz_AS4,3*halfBeat}, {buzz_F4,3*halfBeat}, {buzz_DS4,2*halfBeat}, {buzz_D4,3*halfBeat}, {buzz_DS4,3*halfBeat},
						{buzz_D4,2*halfBeat}, {buzz_C4,3*halfBeat}, {buzz_F4,3*halfBeat}, {buzz_G4,2*halfBeat}, {buzz_A4,3*halfBeat}, {buzz_C5,3*halfBeat}, {buzz_D5,2*halfBeat},
						{buzz_DS5,2*halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,2*halfBeat}, {buzz_D5,halfBeat}, {buzz_DS5,5*halfBeat}, {buzz_Si,halfBeat}, {buzz_DS5,2*halfBeat},
						{buzz_F5,2*halfBeat}, {buzz_G5,2*halfBeat}, {buzz_F5,halfBeat}, {buzz_DS5,2*halfBeat}, {buzz_F5,halfBeat}, {buzz_G5,8*halfBeat}, {buzz_Si,2*halfBeat},
						{buzz_A5,2*halfBeat}, {buzz_FS5,halfBeat}, {buzz_DS5,halfBeat}, {buzz_A4,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_DS5,2*halfBeat},
						{buzz_DS5,2*halfBeat}, {buzz_C5,halfBeat}, {buzz_F5,2*halfBeat}, {buzz_DS5,2*halfBeat}, {buzz_D5,8*halfBeat}, {buzz_Si,halfBeat}, {buzz_G4,halfBeat},
						{buzz_B4,halfBeat}, {buzz_D5,halfBeat}, {buzz_G5,halfBeat}, {buzz_F5,halfBeat}, {buzz_D5,halfBeat}, {buzz_B4,halfBeat}, {buzz_GS5,2*halfBeat},
						{buzz_G5,halfBeat}, {buzz_Si,halfBeat}, {buzz_FS5,2*halfBeat}, {buzz_G5,halfBeat}, {buzz_Si,halfBeat}, {buzz_GS5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat},
						{buzz_C5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat}, {buzz_G5,0.5*halfBeat}, {buzz_D5,0.5*halfBeat}, {buzz_B4,0.5*halfBeat}, {buzz_D5,0.5*halfBeat},
						{buzz_FS5,0.5*halfBeat}, {buzz_CS5,0.5*halfBeat}, {buzz_AS4,0.5*halfBeat}, {buzz_CS5,0.5*halfBeat}, {buzz_G5,0.5*halfBeat}, {buzz_Si,0.5*halfBeat},
						{buzz_C6,0.5*halfBeat}, {buzz_G5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat}, {buzz_G5,0.5*halfBeat}, {buzz_AS5,0.5*halfBeat}, {buzz_F5,0.5*halfBeat},
						{buzz_D5,0.5*halfBeat}, {buzz_F5,0.5*halfBeat}, {buzz_GS5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat}, {buzz_C5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat},
						{buzz_G5,0.5*halfBeat}, {buzz_D5,0.5*halfBeat}, {buzz_B4,0.5*halfBeat}, {buzz_D5,0.5*halfBeat}, {buzz_F5,0.5*halfBeat}, {buzz_C5,0.5*halfBeat},
						{buzz_AS4,0.5*halfBeat}, {buzz_C5,0.5*halfBeat}, {buzz_DS5,0.5*halfBeat}, {buzz_C5,0.5*halfBeat}, {buzz_G4,0.5*halfBeat}, {buzz_C5,0.5*halfBeat},
						{buzz_DS5,0.5*halfBeat}, {buzz_C5,0.5*halfBeat}, {buzz_FS4,0.5*halfBeat}, {buzz_C5,0.5*halfBeat}, {buzz_D5,0.5*halfBeat}, {buzz_B4,0.5*halfBeat},
						{buzz_G4,0.5*halfBeat}, {buzz_D4,0.5*halfBeat}, {buzz_C5,halfBeat}, {buzz_C5,halfBeat}, {buzz_Si,halfBeat}, {buzz_C5,halfBeat}, {buzz_DS5,halfBeat},
						{buzz_C5,halfBeat}, {buzz_Si,halfBeat}, {buzz_C5,halfBeat}, {buzz_DS5,halfBeat}, {buzz_C5,halfBeat}, {buzz_F5,halfBeat}, {buzz_FS5,halfBeat},
						{buzz_F5,0.6666*halfBeat}, {buzz_FS5,0.6666*halfBeat}, {buzz_F5,0.6666*halfBeat}, {buzz_DS5,halfBeat}, {buzz_C5,halfBeat}, {buzz_AS4,halfBeat},
						{buzz_GS5,3*halfBeat}, {buzz_G5,3*halfBeat}, {buzz_FS5,2*halfBeat}, {buzz_Si,2*halfBeat}, {buzz_DS5,2*halfBeat}, {buzz_D5,2*halfBeat}, {buzz_CS5,2*halfBeat},
						{buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat},
						{buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_B4,halfBeat},
						{buzz_CS4,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat}, {buzz_D5,halfBeat}, {buzz_C5,halfBeat},
						{buzz_D5,halfBeat}, {buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_B4,halfBeat}, {buzz_CS4,halfBeat},
						{buzz_B4,halfBeat}, {buzz_CS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_F4,halfBeat}, {buzz_Si,halfBeat}, {buzz_F4,halfBeat}, {buzz_GS4,halfBeat},
						{buzz_F4,halfBeat}, {buzz_Si,halfBeat}, {buzz_F4,halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_AS4,halfBeat}, {buzz_B4,halfBeat},
						{buzz_AS4,0.6666*halfBeat}, {buzz_B4,0.6666*halfBeat}, {buzz_AS4,0.6666*halfBeat}, {buzz_GS4,halfBeat}, {buzz_F4,halfBeat}, {buzz_DS4,halfBeat},
						{0xFFFF,0}};

song_struct imperialMarch[] =
						{{buzz_A4, 0.50}, {buzz_Si, 0.20}, {buzz_A4, 0.50},
						{buzz_Si, 0.20}, {buzz_A4, 0.50}, {buzz_Si, 0.20}, {buzz_F4, 0.40}, {buzz_Si, 0.05}, {buzz_C5, 0.20}, {buzz_Si, 0.05},
						{buzz_A4, 0.60}, {buzz_Si, 0.10}, {buzz_F4, 0.40}, {buzz_Si, 0.05}, {buzz_C5, 0.20},
						{buzz_Si, 0.05}, {buzz_A4, 0.60}, {buzz_Si, 0.80},
						{buzz_E5, 0.50}, {buzz_Si, 0.20},
						{buzz_E5, 0.50}, {buzz_Si, 0.20}, {buzz_E5, 0.50}, {buzz_Si, 0.20}, {buzz_F5, 0.40},
						{buzz_Si, 0.05}, {buzz_C5, 0.20}, {buzz_Si, 0.05},
						{buzz_A4, 0.60}, {buzz_Si, 0.10}, {buzz_F4, 0.40}, {buzz_Si, 0.05}, {buzz_C5, 0.20},
						{buzz_Si, 0.05}, {buzz_A4, 0.60}, {buzz_Si, 0.80}, {0xFFFF, 0}};

song_struct flatLine[] =
						{{buzz_A7,0.10}, {buzz_Si,1.00}, {buzz_A7,0.10}, {buzz_Si,1.00}, {buzz_A7,0.10}, {buzz_Si,1.00}, {buzz_A7,0.10}, {buzz_Si,1.00}, {buzz_A7,0.10}, {buzz_Si,1.00},
						{buzz_A7,0.10}, {buzz_Si,0.10}, {buzz_A7,0.10}, {buzz_Si,0.10}, {buzz_A7,0.10}, {buzz_Si,0.10}, {buzz_A7,0.10}, {buzz_Si,0.10}, {buzz_A7,0.10}, {buzz_Si,0.10},
						{buzz_A7,5.0}, {0xFFFF, 0}};

#endif /* BUZZER_TUNES_H_ */
