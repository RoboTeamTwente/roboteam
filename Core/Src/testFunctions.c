/*
 * testFunctions.c
 *
 *  Created on: May 29, 2019
 *      Author: simen
 */

#include "testFunctions.h"

///////////////////////////////////////////////////// VARIABLES

bool runningTest[nTests] = {false};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

void test_ExecuteFullTest(ReceivedData* receivedData);

void test_ExecuteSquareDrive(ReceivedData* receivedData);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void test_Update(ReceivedData* receivedData) {
	if (runningTest[full]) {
		test_ExecuteFullTest(receivedData);
	} else if (runningTest[square]) {
		test_ExecuteSquareDrive(receivedData);
	}
}

void test_RunTest(tests t) {
	runningTest[t] = true;
}

bool test_isTestRunning() {
	bool result = false;
	for (int i = 0; i < nTests; i++) {
		if (runningTest[i]) {
			result = true;
		}
	}
	return result;
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

void test_ExecuteFullTest(ReceivedData* receivedData) {
	runningTest[full] = true;
	receivedData->do_chip = false;
	receivedData->do_kick = false;
	receivedData->dribblerRef = 0;
	receivedData->genevaRef = geneva_none;
	receivedData->shootPower = 20;
	receivedData->stateRef[body_x] = 0.0f;
	receivedData->stateRef[body_y] = 0.0f;
	receivedData->stateRef[body_w] = 0.0f;
	receivedData->visionAvailable = false;
	receivedData->visionYaw = 0.0f;

	static int start = -100000;
	int timeDiff = HAL_GetTick() - start;
	static int prevTimeDiff = 0;

	if (timeDiff < 1000) {
		if (prevTimeDiff > 21000) {
			Putty_printf("Testing geneva...\n\r");
		}
		receivedData->genevaRef = geneva_leftleft;
	} else if (timeDiff < 2000) {
		receivedData->genevaRef = geneva_left;
	} else if (timeDiff < 3000) {
		receivedData->genevaRef = geneva_middle;
	} else if (timeDiff < 4000) {
		receivedData->genevaRef = geneva_right;
	} else if (timeDiff < 5000) {
		receivedData->genevaRef = geneva_rightright;
	} else if (timeDiff < 10000) {
		if (prevTimeDiff < 5000) {
			Putty_printf("Testing forward driving...\n\r");
		}
		receivedData->stateRef[body_x] = 0.5f;
	} else if (timeDiff < 15000) {
		if (prevTimeDiff < 10000) {
			Putty_printf("Testing sideways driving...\n\r");
		}
		receivedData->stateRef[body_y] = 0.5f;
	} else if (timeDiff < 16000){
		// wait
		if (prevTimeDiff < 15000) {
			Putty_printf("Testing kick...\n\r");
		}
	} else if (timeDiff < 16010) {
		receivedData->do_kick = true;
	} else if (timeDiff < 17000) {
		// wait
		if (prevTimeDiff < 16010) {
			Putty_printf("Testing chip...\n\r");
		}
	} else if (timeDiff < 17010) {
		receivedData->do_chip = true;
	} else if (timeDiff < 18000) {
		// wait
	} else if (timeDiff < 19000) {
		if (prevTimeDiff < 18000) {
			Putty_printf("Testing dribbler...\n\r");
		}
		receivedData->dribblerRef = 50;
	} else if (timeDiff < 21000){
		start = HAL_GetTick();
		runningTest[full] = false;
		Putty_printf("---------- End of test ----------\n\r");
	} else {
		start = HAL_GetTick();
		Putty_printf("---------- Start test! ----------\n\r");
	}
	prevTimeDiff = timeDiff;
}

void test_ExecuteSquareDrive(ReceivedData* receivedData) {
	// SQUARE WITH 90 DEGREES TURNS AT SIDES
	runningTest[square] = true;
	float velocityRef[3];
	velocityRef[0] = 0.0;
	velocityRef[1] = 0.0;
	velocityRef[2] = 0.0*M_PI;

	static int velTimer = 0;
	static int count = 0;
	float v = 0.5;
	int reps = 1;
	int t = 1500;
	if (HAL_GetTick() - velTimer < t) {
		velocityRef[body_x] = v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_w] = 0.0;
	} else if (HAL_GetTick() - velTimer < 2*t) {
		velocityRef[body_x] = v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_w] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 3*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = v;
		velocityRef[body_w] = 0.0;
	} else if (HAL_GetTick() - velTimer < 4*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = v;
		velocityRef[body_w] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 5*t) {
		velocityRef[body_x] = -v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_w] = 0.0;
	} else if (HAL_GetTick() - velTimer < 6*t) {
		velocityRef[body_x] = -v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_w] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 7*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = -v;
		velocityRef[body_w] = 0.0;
	} else if (HAL_GetTick() - velTimer < 8*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = -v;
		velocityRef[body_w] = 0.5*M_PI;
	} else if (count < reps-1) {
		velTimer = HAL_GetTick();
		count++;
	} else if (HAL_GetTick() - velTimer > 10*t) {
		velTimer = HAL_GetTick();
		Putty_printf("---------- Start test! ----------\n\r");
	} else {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = 0.0;
		velocityRef[body_w] = 0.0;
		runningTest[square] = false;
		Putty_printf("---------- End of test ----------\n\r");
	}

	receivedData->stateRef[body_x] = velocityRef[body_x];
	receivedData->stateRef[body_y] = velocityRef[body_y];
	receivedData->stateRef[body_w] = velocityRef[body_w];
//	stateControl_SetRef(velocityRef);
}
