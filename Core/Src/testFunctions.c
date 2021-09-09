/*
 * testFunctions.c
 *
 *  Created on: May 29, 2019
 *      Author: simen
 */

#include "main.h"
#include "testFunctions.h"

///////////////////////////////////////////////////// VARIABLES

#define WHEEL_TEST_SPEED 50.0f
bool runningTest[nTests] = {false};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

status executeFullTest(ReceivedData* receivedData);
status executeWheelsTest();
status executeShootTest();
status executeDribblerTest(ReceivedData* receivedData);
status executeSquareDrive(ReceivedData* receivedData);

void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]);
void checkWheels(int avgPWM[4], int cnt[4], float wheelEncoders[4]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void test_Update(ReceivedData* receivedData) {
	if (runningTest[full]) {
		runningTest[full] = (executeFullTest(receivedData) == test_running);
	} else if (runningTest[square]) {
		runningTest[square] = executeSquareDrive(receivedData) == test_running;
	} else if (runningTest[wheels]) {
		runningTest[wheels] = executeWheelsTest(receivedData) == test_running;
	} else if (runningTest[shoot]) {
		runningTest[shoot] = executeShootTest(receivedData) == test_running;
	} else if (runningTest[dribbler]) {
		runningTest[dribbler] = executeDribblerTest(receivedData) == test_running;
	}
}

void test_RunTest(tests t) {
	runningTest[t] = true;
	Putty_printf("---------- Start test! ----------\n\r");
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

status executeFullTest(ReceivedData* receivedData) {
	receivedData->do_chip = false;
	receivedData->do_kick = false;
	receivedData->kick_chip_forced = false;
	receivedData->dribblerRef = 0;
	receivedData->shootPower = 20;
	receivedData->stateRef[body_x] = 0.0f;
	receivedData->stateRef[body_y] = 0.0f;
	receivedData->stateRef[body_w] = 0.0f;
	receivedData->visionAvailable = false;
	receivedData->visionYaw = 0.0f;

	static status progress[nTests] = {test_none, test_none, test_none, test_none, test_none, test_none};

	if (progress[wheels] == test_running) {
		progress[wheels] = executeWheelsTest();
		if (progress[wheels] == test_done) {
			progress[shoot] = test_running;
		}
	} else if (progress[shoot] == test_running) {
		progress[shoot] = executeShootTest();
		if (progress[shoot] == test_done) {
			progress[dribbler] = test_running;
		}
	} else if (progress[dribbler] == test_running) {
		progress[dribbler] = executeDribblerTest(receivedData);
	} else {
		Putty_printf("---------- End of test ----------\n\r");
		for (int i = 0; i < nTests; i++) {
			progress[i] = test_none;
		}
		return test_done;
	}

	return test_running;
}

status executeWheelsTest() {
	const int RUN_TIME = 2000; // [ticks]
	const int PAUSE_TIME = 1000; // [ticks]
	static uint32_t timer = 0;
	static uint32_t prevTimeDiff = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint32_t timeDiff = HAL_GetTick() - timer;
	if (firstTime) {
		Putty_printf("Testing wheels...\n\r");
		firstTime = false;
	}

	static int avgPWM[4] = {0};
	static int cnt[4] = {0};
	float wheelRef[4] = {0.0f};
	static float wheelEncoders[4] = {0.0f};

	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		uint32_t startTime = wheel * (RUN_TIME + PAUSE_TIME);
		if (timeDiff > startTime && timeDiff < startTime + RUN_TIME) {
			recordWheelData(wheel, avgPWM, cnt, wheelEncoders, wheelRef);
			break;
		}
	}
	wheels_SetRef(wheelRef);

	if (timeDiff >= 4 * (RUN_TIME + PAUSE_TIME) && prevTimeDiff < 4 * (RUN_TIME + PAUSE_TIME)) {
		checkWheels(avgPWM, cnt, wheelEncoders);
		timer = 0;
		prevTimeDiff = 0;
		firstTime = true;
		for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
			avgPWM[wheel] = 0;
			cnt[wheel] = 0;
			wheelEncoders[wheel] = 0;
		}
		return test_done;
	} else {
		prevTimeDiff = timeDiff;
		return test_running;
	}
}

status executeShootTest() {
	const int RUN_TIME = 10; 		// [ticks]
	const int PAUSE_TIME = 1000; 	// [ticks]
	const int MIN_CHARGE_TIME = 10; // [ticks]
	static int chargeCount = 0;
	static uint32_t timer = 0;
	static uint32_t prevTimeDiff = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint32_t timeDiff = HAL_GetTick() - timer;

	// Run test
	if (timeDiff < RUN_TIME) {
		shoot_Shoot(shoot_Kick);
	} else if (timeDiff < RUN_TIME + PAUSE_TIME) {
		chargeCount = (shoot_GetState() == shoot_Charging) ? chargeCount + 1 : chargeCount;
	} else if (timeDiff < 2 * RUN_TIME + PAUSE_TIME) {
		shoot_Shoot(shoot_Chip);
	} else if (timeDiff < 2* (RUN_TIME + PAUSE_TIME)) {
		chargeCount = (shoot_GetState() == shoot_Charging) ? chargeCount + 1 : chargeCount;
	}

	// Print status and results
	if (firstTime) {
		Putty_printf("Testing kick...\n\r");
		firstTime = false;
	} else if (timeDiff >= RUN_TIME + PAUSE_TIME && prevTimeDiff < RUN_TIME + PAUSE_TIME) {
		Putty_printf("\t %s\n\r", chargeCount >= MIN_CHARGE_TIME ? "PASS" : "FAIL");
		chargeCount = 0;
		Putty_printf("Testing chip...\n\r");
	} else if (timeDiff >= 2 * (RUN_TIME + PAUSE_TIME) && prevTimeDiff < 2 * (RUN_TIME + PAUSE_TIME)) {
		Putty_printf("\t %s\n\r", chargeCount >= MIN_CHARGE_TIME ? "PASS" : "FAIL");
		chargeCount = 0;
	}

	if (timeDiff >= 3 * RUN_TIME + 2 * PAUSE_TIME) {
		chargeCount = 0;
		timer = 0;
		prevTimeDiff = 0;
		firstTime = true;
		return test_done;
	} else {
		prevTimeDiff = timeDiff;
		return test_running;
	}
}

status executeDribblerTest(ReceivedData* receivedData) {
	const int RUN_TIME = 2000; 		// [ticks]
	const int PAUSE_TIME = 1000; 	// [ticks]
	static uint32_t timer = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint32_t timeDiff = HAL_GetTick() - timer;

	if (firstTime) {
		Putty_printf("Testing dribbler...\n\r");
		firstTime = false;
	} else if (timeDiff < RUN_TIME) {
		receivedData->dribblerRef = 50;
	}

	if (timeDiff > RUN_TIME + PAUSE_TIME) {
		timer = 0;
		firstTime = true;
		return test_done;
	} else {
		return test_running;
	}
}

status executeSquareDrive(ReceivedData* receivedData) {
	// SQUARE WITH 90 DEGREES TURNS AT SIDES
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
		Putty_printf("---------- End of test ----------\n\r");
		return test_done;
	}

	receivedData->stateRef[body_x] = velocityRef[body_x];
	receivedData->stateRef[body_y] = velocityRef[body_y];
	receivedData->stateRef[body_w] = velocityRef[body_w];

	return test_running;
}

void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]) {
	avgPWM[wheel] += fabs(wheels_GetPWM()[wheel]);
	cnt[wheel]++;
	wheelRef[wheel] = WHEEL_TEST_SPEED;
	wheelEncoders[wheel] += fabs(wheels_GetState()[wheel]);
}

void checkWheels(int avgPWM[4], int cnt[4], float wheelEncoders[4]) {
	int margin = 100; // margin to check if wheel is turning heavily

	int minPWM = MAX_PWM;
	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		avgPWM[wheel] = avgPWM[wheel] / cnt[wheel];
		wheelEncoders[wheel] = wheelEncoders[wheel] / cnt[wheel];
		if (avgPWM[wheel] < minPWM) {
			minPWM = avgPWM[wheel];
		}
	}

	Putty_printf("\t Wheel RF: %s (PWM = %d)\n\r", fabs(wheelEncoders[wheels_RF]) < 5 ? "FAIL" : ((avgPWM[wheels_RF] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_RF]);
	Putty_printf("\t Wheel RB: %s (PWM = %d)\n\r", fabs(wheelEncoders[wheels_RB]) < 5 ? "FAIL" : ((avgPWM[wheels_RB] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_RB]);
	Putty_printf("\t Wheel LB: %s (PWM = %d)\n\r", fabs(wheelEncoders[wheels_LB]) < 5 ? "FAIL" : ((avgPWM[wheels_LB] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_LB]);
	Putty_printf("\t Wheel LF: %s (PWM = %d)\n\r", fabs(wheelEncoders[wheels_LF]) < 5 ? "FAIL" : ((avgPWM[wheels_LF] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_LF]);
}


