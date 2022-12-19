
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

/**
 * Updates the Kalman filter with the newest acceleration and velocity data
 * @param acc The acceleration in X and Y directions as given by the IMU.
 * @param vel The velocity in X and Y directions as measured by the wheels. 
 */
void kalman_Update(float acc[2], float vel[2]);

/**
 * Calculates the kalman gain. 
 */
void kalman_CalculateK();

/**
 * Retrieve the state as determined by the kalman filter. 
 */
void kalman_GetState(float state[4]);

/**
 * Get the kalman gain .
 * @deprecated Seems to not being used 
 */
void kalman_GetK(float gain[4][4]);

/**
 * Get the prediction matrix used by the kalman filter
 * @deprecated Seems to not being used 
 */
void kalman_GetP(float P[16]);

#endif /* KALMAN_KALMAN_H_*/
