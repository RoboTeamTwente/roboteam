/*
 * MTi.h
 *
 *  Created on: 26 okt. 2018
 *      Author: Cas Doornkamp
 */

#ifndef MTI_MTI_H_
#define MTI_MTI_H_

// Includes
#include <stdlib.h>
#include "main.h"
#include "usart.h"
#include "stdint.h"
#include "xbus/xbusparser.h"

// data struct
typedef struct MTi_data_struct{

};


// Error enum
enum{
	Xsens_OK,
	Xsens_Failed_Init
}Xsens_error;



// public function declarations
Xsens_error MTi_Init(*MTi_data_struct);
Xsens_error MTi_DeInit(*MTi_data_struct);
Xsens_error MTi_Update(*MTi_data_struct);
Xsens_error MTi_Reset(*MTi_data_struct);

// private function declarations



#endif /* MTI_MTI_H_ */
