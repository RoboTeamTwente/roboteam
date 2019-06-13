/*
 * testFunctions.c
 *
 *  Created on: May 29, 2019
 *      Author: simen
 */

#include "testFunctions.h"

///////////////////////////////////////////////////// VARIABLES

#define WHEEL_TEST_SPEED 50.0f
bool runningTest[nTests] = {false};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

status executeFullTest(ReceivedData* receivedData);
status executeGenevaTest(ReceivedData* receivedData);
status executeWheelsTest();
status executeShootTest();
status executeDribblerTest(ReceivedData* receivedData);
status executeSquareDrive(ReceivedData* receivedData);

void checkGeneva(geneva_positions position);
void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]);
void checkWheels(int avgPWM[4], int cnt[4], float wheelEncoders[4]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void test_Update(ReceivedData* receivedData) {
	if (runningTest[full]) {
		runningTest[full] = (executeFullTest(receivedData) == test_running);
	} else if (runningTest[square]) {
		runningTest[square] = executeSquareDrive(receivedData) == test_running;
	} else if (runningTest[geneva]) {
		runningTest[geneva] = executeGenevaTest(receivedData) == test_running;
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

//status executeFullTest(ReceivedData* receivedData) {
//	receivedData->do_chip = false;
//	receivedData->do_kick = false;
//	receivedData->kick_chip_forced = false;
//	receivedData->dribblerRef = 0;
//	receivedData->genevaRef = geneva_none;
//	receivedData->shootPower = 20;
//	receivedData->stateRef[body_x] = 0.0f;
//	receivedData->stateRef[body_y] = 0.0f;
//	receivedData->stateRef[body_w] = 0.0f;
//	receivedData->visionAvailable = false;
//	receivedData->visionYaw = 0.0f;
//
//	static int start = -100000;
//	int timeDiff = HAL_GetTick() - start;
//	static int prevTimeDiff = 0;
//
//	if (timeDiff < 1000) {
//		if (prevTimeDiff > 21000) {
//			Putty_printf("Testing geneva...\n\r");
//		}
//		receivedData->genevaRef = geneva_leftleft;
//	} else if (timeDiff < 2000) {
//		if (prevTimeDiff < 1000) {
//			checkGeneva(geneva_leftleft);
//		}
//		receivedData->genevaRef = geneva_left;
//	} else if (timeDiff < 3000) {
//		if (prevTimeDiff < 2000) {
//			checkGeneva(geneva_left);
//		}
//		receivedData->genevaRef = geneva_middle;
//	} else if (timeDiff < 4000) {
//		if (prevTimeDiff < 3000) {
//			checkGeneva(geneva_middle);
//		}
//		receivedData->genevaRef = geneva_right;
//	} else if (timeDiff < 5000) {
//		if (prevTimeDiff < 4000) {
//			checkGeneva(geneva_right);
//		}
//		receivedData->genevaRef = geneva_rightright;
//	} else if (timeDiff < 17000) {
//		if (prevTimeDiff < 5000) {
//			checkGeneva(geneva_rightright);
//			receivedData->genevaRef = geneva_middle;
//			Putty_printf("Testing wheels...\n\r");
//		}
//
//		static int avgPWM[4] = {0};
//		static int cnt[4] = {0};
//		float wheelRef[4] = {0.0f};
//		static float wheelEncoders[4] = {0.0f};
//
//		if (timeDiff > 5500 && timeDiff < 7500) {
//			recordWheelData(wheels_RF, avgPWM, cnt, wheelEncoders, wheelRef);
//		}
//		else if (timeDiff > 8500 && timeDiff < 10500) {
//			recordWheelData(wheels_RB, avgPWM, cnt, wheelEncoders, wheelRef);
//		}
//		else if (timeDiff > 11500 && timeDiff < 13500) {
//			recordWheelData(wheels_LB, avgPWM, cnt, wheelEncoders, wheelRef);
//		}
//		else if (timeDiff > 14500 && timeDiff < 16500) {
//			recordWheelData(wheels_LF, avgPWM, cnt, wheelEncoders, wheelRef);
//		}
//		else if (timeDiff > 16500 && prevTimeDiff <= 16500) {
//			checkWheels(avgPWM, cnt, wheelEncoders);
//		}
//		wheels_SetRef(wheelRef);
//	} else if (timeDiff < 18000){
//		// wait
//		if (prevTimeDiff < 17000) {
//			Putty_printf("Testing kick...\n\r");
//		}
//	} else if (timeDiff < 18010) {
//		shoot_Shoot(shoot_Kick);
//	} else if (timeDiff < 19000) {
//		// wait
//		if (prevTimeDiff < 18010) {
//			Putty_printf("Testing chip...\n\r");
//		}
//	} else if (timeDiff < 19010) {
//		shoot_Shoot(shoot_Chip);
//	} else if (timeDiff < 20000) {
//		// wait
//	} else if (timeDiff < 21000) {
//		if (prevTimeDiff < 20000) {
//			Putty_printf("Testing dribbler...\n\r");
//		}
//		receivedData->dribblerRef = 50;
//	} else if (timeDiff < 23000){
//		start = HAL_GetTick();
//		Putty_printf("---------- End of test ----------\n\r");
//		return done;
//	} else {
//		start = HAL_GetTick();
//		Putty_printf("---------- Start test! ----------\n\r");
//	}
//	prevTimeDiff = timeDiff;
//	return running;
//}

status executeFullTest(ReceivedData* receivedData) {
	receivedData->do_chip = false;
	receivedData->do_kick = false;
	receivedData->kick_chip_forced = false;
	receivedData->dribblerRef = 0;
	receivedData->genevaRef = geneva_none;
	receivedData->shootPower = 20;
	receivedData->stateRef[body_x] = 0.0f;
	receivedData->stateRef[body_y] = 0.0f;
	receivedData->stateRef[body_w] = 0.0f;
	receivedData->visionAvailable = false;
	receivedData->visionYaw = 0.0f;

	static status progress[nTests] = {test_none, test_none, test_none, test_none, test_none, test_none};

	if (progress[geneva] == test_running) {
		progress[geneva] = executeGenevaTest(receivedData);
		if (progress[geneva] == test_done) {
			progress[wheels] = test_running;
		}
	} else if (progress[wheels] == test_running) {
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
	} else if (progress[geneva] == test_none) {
		Putty_printf("start with geneva \n\r");
		progress[geneva] = test_running; // start with geneva test
	} else {
		Putty_printf("---------- End of test ----------\n\r");
		return test_done;
	}

	return test_running;
}

status executeGenevaTest(ReceivedData* receivedData) {

	const int RUN_TIME = 1000; 		// [ticks]
	static uint timer = 0;
	static uint prevTimeDiff = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint timeDiff = HAL_GetTick() - timer;
	if (firstTime) {
		Putty_printf("Testing geneva...\n\r");
		firstTime = false;
	}

	for (geneva_positions pos = geneva_leftleft; pos <= geneva_rightright + 1; pos++) {
		if (timeDiff < pos * RUN_TIME) {
			if (pos > geneva_leftleft && prevTimeDiff < (pos - 1) * RUN_TIME) {
				checkGeneva(pos - 1);
			}
			receivedData->genevaRef = pos > geneva_rightright ? geneva_middle : pos;
			break;
		}
	}

	if (timeDiff >= 6 * RUN_TIME && prevTimeDiff < 6 * RUN_TIME) {
		return test_done;
	} else {
		prevTimeDiff = timeDiff;
		return test_running;
	}
}

status executeWheelsTest() {
	const int RUN_TIME = 2000; // [ticks]
	const int PAUSE_TIME = 1000; // [ticks]
	static uint timer = 0;
	static uint prevTimeDiff = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint timeDiff = HAL_GetTick() - timer;
	if (firstTime) {
		Putty_printf("Testing wheels...\n\r");
		firstTime = false;
	}

	static int avgPWM[4] = {0};
	static int cnt[4] = {0};
	float wheelRef[4] = {0.0f};
	static float wheelEncoders[4] = {0.0f};

	for (wheel_names wheel = wheels_RF; wheel <= wheels_LF; wheel++) {
		uint startTime = wheel * (RUN_TIME + PAUSE_TIME);
		if (timeDiff > startTime && timeDiff < startTime + RUN_TIME) {
			recordWheelData(wheel, avgPWM, cnt, wheelEncoders, wheelRef);
			break;
		}
	}
	wheels_SetRef(wheelRef);

	if (timeDiff >= 4 * (RUN_TIME + PAUSE_TIME) && prevTimeDiff < 4 * (RUN_TIME + PAUSE_TIME)) {
		checkWheels(avgPWM, cnt, wheelEncoders);
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
	static uint timer = 0;
	static uint prevTimeDiff = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint timeDiff = HAL_GetTick() - timer;

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
		return test_done;
	} else {
		prevTimeDiff = timeDiff;
		return test_running;
	}
}

status executeDribblerTest(ReceivedData* receivedData) {
	const int RUN_TIME = 2000; 		// [ticks]
	const int PAUSE_TIME = 1000; 	// [ticks]
	static uint timer = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint timeDiff = HAL_GetTick() - timer;

	if (firstTime) {
		Putty_printf("Testing dribbler...\n\r");
		firstTime = false;
	} else if (timeDiff < RUN_TIME) {
		receivedData->dribblerRef = 50;
	}

	if (timeDiff > RUN_TIME + PAUSE_TIME) {
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

void checkGeneva(geneva_positions position) {
	int margin = 20; // if geneva within 5 encoder units, test passes

	int encoderDiff = fabs(encoderForPosition[position] - geneva_GetEncoder());
	Putty_printf("\t position %d: %s (offset: %d encoder units)\n\r", position, (encoderDiff < margin) ? "PASS" : "FAIL", encoderDiff);
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


