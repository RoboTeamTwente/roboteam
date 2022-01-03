
/* Description: Kalman filter
 *
 * Instructions:
 * 1) See the wikipedia page on kalman filters
 * 2) Takes data from different sensors, combines them and smoothes
 * 3) First calculate K and P which after a certain iterations should be constant
 * 4) Then use K to filter the sensor data
 *
 * Extra functions:
 *
 * Notes:
 * Don't use pointers with Kalman it fucks shit up
 *
*/

#ifndef KALMAN_KALMAN_H_
#define KALMAN_KALMAN_H_

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void kalman_Init();

void kalman_DeInit();

void kalman_Update(float acc[2], float vel[2]);

void kalman_CalculateK();

void kalman_GetState(float state[4]);

// seems to be not used
void kalman_GetK(float gain[4][4]);

// seems to be not used
void kalman_GetP(float P[16]);

#endif /* KALMAN_KALMAN_H_*/
