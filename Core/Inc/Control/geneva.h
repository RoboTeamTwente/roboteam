
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
