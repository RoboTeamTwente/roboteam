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

void kalman_Init();
void kalman_Deinit();
void kalman_Update(float acc[2], float vel[2]);
void kalman_CalculateK();
void kalman_GetState(float state[4]);
void kalman_GetGain(float gain[4][4]);
void kalman_GetP(float P[16]);
#endif /* KALMAN_KALMAN_H_*/
