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
#include "Buzzer/buzzer.h"
#include "dribbler.h"

///////////////////////////////////////////////////// DEFINITIONS

#define nTests 7

typedef enum {
	normal,
	full,
	square,
	wheels,
	shoot,
	dribbler,
	any
} tests;

typedef enum {
	test_done,
	test_running,
	test_none
} status;

///////////////////////////////////////////////////// PUBLIC FUNCTION DECLARATIONS

void test_Update(ReceivedData* receivedData);

bool test_isTestRunning(tests t);

void test_RunTest(tests t);

void test_StopTest(tests t);

void test_Buzzer(const char *song);

#endif /* INC_TESTFUNCTIONS_H_ */
