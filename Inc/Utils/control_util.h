/*
 * control_util.h
 *
 *  Created on: Nov 14, 2018
 *      Author: kjhertenberg
 */

#ifndef UTILS_CONTROL_UTIL_H_
#define UTILS_CONTROL_UTIL_H_

typedef enum {
	body_x,
	body_y,
	body_w,
}body_handles;

typedef enum {
	wheels_RF,
	wheels_RB,
	wheels_LB,
	wheels_LF,
}wheel_names;

typedef struct {
	float kP;
	float kI;
	float kD;
	float prev_e;
	float timeDiff;
}PIDvariables;

//PID control, static to not have multiple implmentation error
static float PID(float err, PIDvariables K){
	static float I = 0;
	float P = K.kP*err;
	I += K.kI*err*K.timeDiff;
	float D = (K.kD*(err-K.prev_e))/K.timeDiff;
	K.prev_e = err;
	float PIDvalue = P + I + D;
	return PIDvalue;
}

#endif /* UTILS_CONTROL_UTIL_H_ */
