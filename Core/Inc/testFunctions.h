/*
 * testFunctions.h
 *
 *  Created on: May 29, 2019
 *      Author: simen
 */

#ifndef INC_TESTFUNCTIONS_H_
#define INC_TESTFUNCTIONS_H_

#include <stdbool.h>
#include "packing.h"
#include "stm32f7xx_hal.h"
#include "PuTTY.h"
#include "stateControl.h"

///////////////////////////////////////////////////// DEFINITIONS

#define nTests 2

typedef enum {
	full,
	square
} tests;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void test_Update(ReceivedData* receivedData);

bool test_isTestRunning();

void test_RunTest(tests t);

#endif /* INC_TESTFUNCTIONS_H_ */
