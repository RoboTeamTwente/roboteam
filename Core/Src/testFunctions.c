/*
 * testFunctions.c
 *
 *  Created on: May 29, 2019
 *      Author: simen
 */

#include "main.h"
#include "testFunctions.h"
#include <string.h>
#include "shoot.h"
#include "logging.h"

///////////////////////////////////////////////////// VARIABLES

#define WHEEL_TEST_SPEED 50.0f
bool runningTest[nTests] = {false};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

status executeFullTest();
status executeWheelsTest();
status executeShootTest();
status executeDribblerTest();
status executeSquareDrive();

void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]);
void checkWheels(int avgPWM[4], int cnt[4], float wheelEncoders[4]);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

void test_Update() {
	if (runningTest[full]) {
		runningTest[full] = (executeFullTest() == test_running);
	} else if (runningTest[square]) {
		runningTest[square] = executeSquareDrive() == test_running;
	} else if (runningTest[wheels]) {
		runningTest[wheels] = executeWheelsTest() == test_running;
	} else if (runningTest[shoot]) {
		runningTest[shoot] = executeShootTest() == test_running;
	} else if (runningTest[dribbler]) {
		runningTest[dribbler] = executeDribblerTest() == test_running;
	}
}

void test_RunTest(tests t) {
	runningTest[t] = true;
	LOG("---------- Start test! ----------\n");
}

void test_StopTest(tests t) {
    runningTest[t] = false;
    LOG("---------- End of test ----------\n");
}

bool test_isTestRunning(tests t) {
    if (t == any) {
        bool result = false;
        for (int i = 0; i < nTests; i++) {
            if (runningTest[i]) {
                result = true;
            }
        }
        return result;
    }
    else {
        return runningTest[t];
    }
}

void test_Buzzer(const char *song) {
    if (!strcmp(song, "quick beep up")) {
        buzzer_Play_QuickBeepUp();
    } else if (!strcmp(song, "quick beep down")) {
        buzzer_Play_QuickBeepDown();
    } else if (!strcmp(song, "startup")) {
        buzzer_Play_Startup();
    } else if (!strcmp(song, "tetris")) {
        buzzer_Play_Tetris();
    } else if (!strcmp(song, "mario")) {
        buzzer_Play_Mario();
    } else if (!strcmp(song, "pink panther")) {
        buzzer_Play_PinkPanther();
    } else if (!strcmp(song, "power up")) {
        buzzer_Play_PowerUp();
    } else if (!strcmp(song, "warning one")) {
        buzzer_Play_WarningOne();
    } else if (!strcmp(song, "warning two")) {
        buzzer_Play_WarningTwo();
    } else if (!strcmp(song, "warning three")) {
        buzzer_Play_WarningThree();
    } else if (!strcmp(song, "warning four")) {
        buzzer_Play_WarningFour();
    } else if (!strcmp(song, "bridge battle")) {
        buzzer_Play_BridgeBattle();
    } else if (!strcmp(song, "imperial march")) {
        buzzer_Play_ImperialMarch();
    } else if (!strcmp(song, "flatline")) {
        buzzer_Play_Flatline();
    } else if (!strcmp(song, "happy birthday")) {
        buzzer_Play_HBD();
    }
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

status executeFullTest() {
	static status progress[nTests] = {test_none, test_none, test_none, test_none, test_none, test_none};

    if (progress[wheels] == test_none) {
        progress[wheels] = test_running;
        runningTest[wheels] = true;
    } else if (progress[wheels] == test_running) {
		progress[wheels] = executeWheelsTest();
		if (progress[wheels] == test_done) {
			progress[shoot] = test_running;
            runningTest[wheels] = false;
            runningTest[shoot] = true;
		}
	} else if (progress[shoot] == test_running) {
		progress[shoot] = executeShootTest();
		if (progress[shoot] == test_done) {
			progress[dribbler] = test_running;
            runningTest[shoot] = false;
            runningTest[dribbler] = true;
		}
	} else if (progress[dribbler] == test_running) {
		progress[dribbler] = executeDribblerTest();
	} else {
		LOG("---------- End of test ----------\n");
		for (int i = 0; i < nTests; i++) {
			progress[i] = test_none;
		}
		runningTest[dribbler] = false;
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
		LOG("Testing wheels...\n");
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
	wheels_SetSpeeds(wheelRef);

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
        LOG("---------- End of test ----------\n");
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
		LOG("Testing kick...\n");
		firstTime = false;
	} else if (timeDiff >= RUN_TIME + PAUSE_TIME && prevTimeDiff < RUN_TIME + PAUSE_TIME) {
		LOG_printf("\t %s\n", chargeCount >= MIN_CHARGE_TIME ? "PASS" : "FAIL");
		chargeCount = 0;
		LOG("Testing chip...\n");
	} else if (timeDiff >= 2 * (RUN_TIME + PAUSE_TIME) && prevTimeDiff < 2 * (RUN_TIME + PAUSE_TIME)) {
		LOG_printf("\t %s\n", chargeCount >= MIN_CHARGE_TIME ? "PASS" : "FAIL");
		chargeCount = 0;
	}

	if (timeDiff >= 3 * RUN_TIME + 2 * PAUSE_TIME) {
		chargeCount = 0;
		timer = 0;
		prevTimeDiff = 0;
		firstTime = true;
        LOG("---------- End of test ----------\n");
		return test_done;
	} else {
		prevTimeDiff = timeDiff;
		return test_running;
	}
}

status executeDribblerTest() {
	const int RUN_TIME = 2000; 		// [ticks]
	const int PAUSE_TIME = 1000; 	// [ticks]
	static uint32_t timer = 0;
	static bool firstTime = true;
	timer = timer == 0 ? HAL_GetTick() : timer;

	uint32_t timeDiff = HAL_GetTick() - timer;

	if (firstTime) {
		LOG("Testing dribbler...\n");
		firstTime = false;
	} else if (timeDiff < RUN_TIME) {
		dribbler_SetSpeed(50);
	}

	if (timeDiff > RUN_TIME + PAUSE_TIME) {
		timer = 0;
		firstTime = true;
        LOG("---------- End of test ----------\n");
        dribbler_SetSpeed(0);
		return test_done;
	} else {
		return test_running;
	}
}

status executeSquareDrive() {
	// SQUARE WITH 90 DEGREES TURNS AT SIDES
	float velocityRef[4];
	velocityRef[0] = 0.0;
	velocityRef[1] = 0.0;
	velocityRef[2] = 0.0 * M_PI;

	static int velTimer = 0;
	static int count = 0;
	float v = 0.5;
	int reps = 1;
	int t = 1500;
	if (HAL_GetTick() - velTimer < t) {
		velocityRef[body_x] = v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_yaw] = 0.0;
	} else if (HAL_GetTick() - velTimer < 2*t) {
		velocityRef[body_x] = v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_yaw] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 3*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = v;
		velocityRef[body_yaw] = 0.0;
	} else if (HAL_GetTick() - velTimer < 4*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = v;
		velocityRef[body_yaw] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 5*t) {
		velocityRef[body_x] = -v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_yaw] = 0.0;
	} else if (HAL_GetTick() - velTimer < 6*t) {
		velocityRef[body_x] = -v;
		velocityRef[body_y] = 0.0;
		velocityRef[body_yaw] = 0.5*M_PI;
	} else if (HAL_GetTick() - velTimer < 7*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = -v;
		velocityRef[body_yaw] = 0.0;
	} else if (HAL_GetTick() - velTimer < 8*t) {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = -v;
		velocityRef[body_yaw] = 0.5*M_PI;
	} else if (count < reps-1) {
		velTimer = HAL_GetTick();
		count++;
	} else if (HAL_GetTick() - velTimer > 10*t) {
		velTimer = HAL_GetTick();
	} else {
		velocityRef[body_x] = 0.0;
		velocityRef[body_y] = 0.0;
		velocityRef[body_yaw] = 0.0;
		LOG("---------- End of test ----------\n");
		return test_done;
	}

    stateControl_SetRef(velocityRef);
	return test_running;
}

void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]) {
	uint32_t wheel_PWMs[4];
	wheels_GetPWM(wheel_PWMs);
	avgPWM[wheel] += fabs(wheel_PWMs[wheel]);
	cnt[wheel]++;
	wheelRef[wheel] = WHEEL_TEST_SPEED;
	float wheel_measured_speeds[4];
	wheels_GetMeasuredSpeeds(wheel_measured_speeds);
	wheelEncoders[wheel] += fabs(wheel_measured_speeds[wheel]);
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

	LOG_printf("\t Wheel RF: %s (PWM = %d)\n", fabs(wheelEncoders[wheels_RF]) < 5 ? "FAIL" : ((avgPWM[wheels_RF] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_RF]);
	LOG_printf("\t Wheel RB: %s (PWM = %d)\n", fabs(wheelEncoders[wheels_RB]) < 5 ? "FAIL" : ((avgPWM[wheels_RB] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_RB]);
	LOG_printf("\t Wheel LB: %s (PWM = %d)\n", fabs(wheelEncoders[wheels_LB]) < 5 ? "FAIL" : ((avgPWM[wheels_LB] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_LB]);
	LOG_printf("\t Wheel LF: %s (PWM = %d)\n", fabs(wheelEncoders[wheels_LF]) < 5 ? "FAIL" : ((avgPWM[wheels_LF] > minPWM + margin) ? "HEAVY" : "PASS"), avgPWM[wheels_LF]);
}


