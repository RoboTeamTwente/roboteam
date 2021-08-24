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

#define nTests 7

typedef enum {
	normal,
	full,
	square,
	wheels,
	shoot,
	dribbler
} tests;

typedef enum {
	test_done,
	test_running,
	test_none
} status;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void test_Update(ReceivedData* receivedData);

bool test_isTestRunning();

void test_RunTest(tests t);

status test_ExecuteWheelsTest();

#endif /* INC_TESTFUNCTIONS_H_ */
