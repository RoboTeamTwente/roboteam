
/* Description: Makes the dribbler spin
 *
 * Instructions:
 * 1) set speed (0(off)-1(max))
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef DRIBBLER_DRIBBLER_H_
#define DRIBBLER_DRIBBLER_H_

#include "control_util.h"
#include "gpio_util.h"
#include "tim_util.h"

typedef struct movingAverage {
    float movingAvgBuffer[5];
    float filteredBuffer[3];
    int movingAvgIdx;
    int filteredIdx;
    float speedBeforeGotBall;
    float commandedSpeed;
} movingAverage;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

// Initializes the PIDs / encoders / PWM timers
void dribbler_Init();
// Denitializes the encoders / PWM timers
void dribbler_DeInit();
// Sets the dribbler speed and makes sure it's within [0,1]
void dribbler_SetSpeed(float speed);
// Updates the dribbler towards the commanded dribbler speed using the encoder and a PID controller.
void dribbler_Update();
// Get the last measured dribbler speeds in rad/s
void dribbler_GetMeasuredSpeeds(float *speed);
// Get the filtered dribbler speeds in rad/s
void dribbler_GetFilteredSpeeds(float *speed);
// returns true if the dribbler speed decreases
bool dribbler_hasBall();

void dribbler_GetSpeedBeforeGotBall(float *speed);

#endif /* DRIBBLER_DRIBBLER_H_ */
