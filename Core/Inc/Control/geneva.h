/*
 * geneva.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Leon
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

#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void geneva_Init();

void geneva_Deinit();

void geneva_Update();

geneva_positions geneva_SetRef(geneva_positions position);

geneva_positions geneva_GetState();

int geneva_GetPWM();

#endif /* GENEVA_GENEVA_H_ */
