/*
 * Kalman.h
 *
 *  Created on: Feb 19, 2019
 *      Author: kjhertenberg
 */

/* Description:
 *
 * Instructions:
 * 1)
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef KALMAN_KALMAN_H_
#define KALMAN_KALMAN_H_

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void kalmanInit();
void kalmanDeinit();
void KalmanState(float acc[2], float vel[2]);
void KalmanK();
void getKState(float state[4]);
void getKGain(float gain[4][4]);
void getP(float P[16]);
#endif /* KALMAN_KALMAN_H_*/
