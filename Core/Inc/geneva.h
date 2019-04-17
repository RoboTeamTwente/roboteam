
/* Description: Controls the position of the Geneva
 *
 * Instructions:
 * 1) First the ref position is moved till we hit the edge, to calibrate the encoder
 * 2) The controller is constantly run
 * 3) The position is set with a function
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef GENEVA_GENEVA_H_
#define GENEVA_GENEVA_H_

#include "../Util/control_util.h"
#include "../Util/gpio_util.h"
#include "../Util/tim_util.h"

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void geneva_Init();

void geneva_Deinit();

void geneva_Update();

void geneva_SetRef(geneva_positions position);

geneva_positions geneva_GetState();

int geneva_GetPWM();

#endif /* GENEVA_GENEVA_H_ */
