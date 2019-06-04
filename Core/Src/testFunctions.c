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

void test_ExecuteFullTest(ReceivedData* receivedData);
void checkGeneva(geneva_positions position);
void recordWheelData(wheel_names wheel, int avgPWM[4], int cnt[4], float wheelEncoders[4], float wheelRef[4]);
void checkWheels(int avgPWM[4], int cnt[4], float wheelEncoders[4]);

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
		if (prevTimeDiff < 1000) {
			checkGeneva(geneva_leftleft);
		}
		receivedData->genevaRef = geneva_left;
	} else if (timeDiff < 3000) {
		if (prevTimeDiff < 2000) {
			checkGeneva(geneva_left);
		}
		receivedData->genevaRef = geneva_middle;
	} else if (timeDiff < 4000) {
		if (prevTimeDiff < 3000) {
			checkGeneva(geneva_middle);
		}
		receivedData->genevaRef = geneva_right;
	} else if (timeDiff < 5000) {
		if (prevTimeDiff < 4000) {
			checkGeneva(geneva_right);
		}
		receivedData->genevaRef = geneva_rightright;
	} else if (timeDiff < 17000) {
		if (prevTimeDiff < 5000) {
			checkGeneva(geneva_rightright);
			receivedData->genevaRef = geneva_middle;
			Putty_printf("Testing wheels...\n\r");
		}

		static int avgPWM[4] = {0};
		static int cnt[4] = {0};
		float wheelRef[4] = {0.0f};
		static float wheelEncoders[4] = {0.0f};

		if (timeDiff > 5500 && timeDiff < 7500) {
			recordWheelData(wheels_RF, avgPWM, cnt, wheelEncoders, wheelRef);
		}
		else if (timeDiff > 8500 && timeDiff < 10500) {
			recordWheelData(wheels_RB, avgPWM, cnt, wheelEncoders, wheelRef);
		}
		else if (timeDiff > 11500 && timeDiff < 13500) {
			recordWheelData(wheels_LB, avgPWM, cnt, wheelEncoders, wheelRef);
		}
		else if (timeDiff > 14500 && timeDiff < 16500) {
			recordWheelData(wheels_LF, avgPWM, cnt, wheelEncoders, wheelRef);
		}
		else if (timeDiff > 16500 && prevTimeDiff <= 16500) {
			checkWheels(avgPWM, cnt, wheelEncoders);
		}
		wheels_SetRef(wheelRef);
	} else if (timeDiff < 18000){
		// wait
		if (prevTimeDiff < 17000) {
			Putty_printf("Testing kick...\n\r");
		}
	} else if (timeDiff < 18010) {
		receivedData->do_kick = true;
	} else if (timeDiff < 19000) {
		// wait
		if (prevTimeDiff < 18010) {
			Putty_printf("Testing chip...\n\r");
		}
	} else if (timeDiff < 19010) {
		receivedData->do_chip = true;
	} else if (timeDiff < 20000) {
		// wait
	} else if (timeDiff < 21000) {
		if (prevTimeDiff < 20000) {
			Putty_printf("Testing dribbler...\n\r");
		}
		receivedData->dribblerRef = 50;
	} else if (timeDiff < 23000){
		start = HAL_GetTick();
		runningTest[full] = false;
		Putty_printf("---------- End of test ----------\n\r");
	} else {
		start = HAL_GetTick();
		Putty_printf("---------- Start test! ----------\n\r");
	}
	prevTimeDiff = timeDiff;
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
