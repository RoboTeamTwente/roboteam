
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

///////////////////////////////////////////////////// VARIABLES

static int encoderForPosition[6] = {0, 1900, 2395, 2890, 3385, 3880};

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void geneva_Init();

void geneva_DeInit();

void geneva_Update();

void geneva_SetRef(geneva_positions position);

float geneva_GetRef();

int geneva_GetEncoder();

int geneva_GetPWM();

#endif /* GENEVA_GENEVA_H_ */
