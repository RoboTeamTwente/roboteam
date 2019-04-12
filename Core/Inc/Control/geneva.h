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


///////////////////////////////////////////////////// STRUCTS



///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void geneva_Init();
void geneva_Deinit();
void geneva_Update();
geneva_positions geneva_SetPosition(geneva_positions position);
geneva_positions geneva_GetPosition();
int getPWM();

#endif /* GENEVA_GENEVA_H_ */
