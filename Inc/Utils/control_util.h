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
	float I;
	float prev_e;
	float timeDiff;
}PIDvariables;

//PID control, static to not have multiple implementation error
inline float PID(float err, PIDvariables* K){
	float P = K.kP*err;
	K.I += K.kI*err*K.timeDiff;
	float D = (K.kD*(err-K.prev_e))/K.timeDiff;
	K.prev_e = err;
	float PIDvalue = P + K.I + D;
	return PIDvalue;
}

#endif /* UTILS_CONTROL_UTIL_H_ */
