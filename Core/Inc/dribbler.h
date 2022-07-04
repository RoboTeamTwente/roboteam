
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

#define sizeOfMovingAverageBuffer 5
#define sizeOfDelay 3
#define sizeOfCommandBuffer 5
#define minReliableData 400.0


typedef struct movingAverage {
    float movingAvgBuffer[sizeOfMovingAverageBuffer]; // stores measured speeds for a moving average filter
    int movingAvgIdx; // index for the moving average buffer
    float commandedSpeedBuffer[sizeOfCommandBuffer];
    int commandedIdx;
    float AvgCommandedSpeed;
    float speedBeforeGotBall;
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
// Returns true if the dribbler speed decreases
bool dribbler_hasBall();
// Returns the delayed speed of the moving average filter at the time it got the ball
void dribbler_GetSpeedBeforeGotBall(float *speed);

#endif /* DRIBBLER_DRIBBLER_H_ */
