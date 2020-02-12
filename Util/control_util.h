/*
 * control_util.h
 *
 *  Created on: Nov 14, 2018
 *      Author: kjhertenberg
 */

/* Description:
 *
 * Instructions:
 * 1)
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef UTILS_CONTROL_UTIL_H_
#define UTILS_CONTROL_UTIL_H_

#include <math.h>
#include <stdbool.h>

///////////////////////////////////////////////////// DEFINITIONS

// System
#define TIME_DIFF 0.01F 		// time difference due to 100Hz frequency
#define WIRELESS_RX_COUNT 4000  // count after which wireless should go to timeout after last packet. Multiply with period base (62.5 us) to get to the time in seconds.

// Robot
#define rad_robot 0.081F 	// robot radius (m) (from center to wheel contact point)
#define rad_wheel 0.028F 	// wheel radius (m)

#define FRONT_ANGLE 30		// angle of front wheels (deg)
#define BACK_ANGLE 60		// angle of back wheels (deg)
#define cosFront cos(FRONT_ANGLE * M_PI/180)
#define sinFront sin(FRONT_ANGLE * M_PI/180)
#define cosBack cos(BACK_ANGLE * M_PI/180)
#define sinBack sin(BACK_ANGLE * M_PI/180)

bool MOTORS_50W;			// wattage of motors, true = 50 W, false = 30 W

// Wheels
#define PWM_CUTOFF 200.0F // arbitrary treshold to avoid motor shutdown
#define GEAR_RATIO 2.5F // gear ratio between motor and wheel
#define MAX_PWM 3000 // defined in CubeMX
#define PWM_LIMIT MAX_PWM // should be equal to MAX_PWM by default
float MAX_VOLTAGE; // [V] see datasheet
#define MAX_VOLTAGE_30W 12.0
#define MAX_VOLTAGE_50W 24.0
float SPEED_CONSTANT; //[(rad/s)/V] see datasheet
#define SPEED_CONSTANT_30W 374.0
#define SPEED_CONSTANT_50W 285.0
#define PULSES_PER_ROTATION (float)4*1024 // number of pulses of the encoder per rotation of the motor (see datasheet)

float OMEGAtoPWM; // conversion factor from wheel speed [rad/s] to required PWM on the motor
#define ENCODERtoOMEGA (float)2*M_PI/(TIME_DIFF*GEAR_RATIO*PULSES_PER_ROTATION) // conversion factor from number of encoder pulses to wheel speed [rad/s]

// Control
#define YAW_MARGIN (0.5F/180.0F)*(float)M_PI // margin at which the I-value of the PID is reset to 0
float WHEEL_REF_LIMIT; // [rad/s] Limit the maximum wheel reference to leave room for the wheels PID
#define WHEEL_REF_LIMIT_PWM 2200 // [pwm]

// Geneva
#define GENEVA_CAL_EDGE_CNT 4100		// the amount of encoder counts from one edge to the other
#define ENCODER_DEVIATION_MARGIN 3		// margin within which encoder is considered to be the same as previous encoder
#define GENEVA_NOT_WORKING_TIME 2	 	// number of seconds not responding after which geneva is considered to be not working

// Shoot
#define MIN_KICK_TIME 25 				// minimum time period of kicking
#define MAX_KICK_TIME 300 				// maximum time period of kicking
#define MIN_CHIP_TIME 60 				// minimum time period of chipping
#define MAX_CHIP_TIME 160 				// maximum time period of chipping
#define TIMER_FREQ 10000 			// frequency [Hz] of TIM6  (Clock frequency divided by prescaler)
#define READY_CALLBACK_FREQ 1 		// frequency [Hz] of callback when shootState is Ready
#define CHARGING_CALLBACK_FREQ 10 	// frequency [Hz] of callback when shootState is Charging
#define SHOOTING_CALLBACK_FREQ 10 	// frequency [Hz] of callback when shootState is Shooting
#define OFF_CALLBACK_FREQ 1 		// frequency [Hz] of callback when shootState is Off

///////////////////////////////////////////////////// STRUCTS

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

typedef enum{
	geneva_none,
	geneva_leftleft,
	geneva_left,
	geneva_middle,
	geneva_right,
	geneva_rightright
}geneva_positions;

typedef enum {
	off,
	setup,
	on,
	turning,
	idle
}PID_states;// keeps track of the state of the system

struct PIDstruct{
	float kP;
	float kI;
	float kD;
	float I;
	float prev_e;
	float timeDiff;
	float minOutput;
	float maxOutput;
	float ramp;
	float prev_PID;
}static PIDdefault = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, TIME_DIFF, -1000000, 1000000, 1000000, 0};

typedef struct PIDstruct PIDvariables;

///////////////////////////////////////////////////// FUNCTIONS
/**
 * Initializes motor wattage dependent constants
 */
inline void control_util_Init() {
	MAX_VOLTAGE = MOTORS_50W ? MAX_VOLTAGE_50W : MAX_VOLTAGE_30W;
	SPEED_CONSTANT = 2*M_PI/60.0 * (MOTORS_50W ? SPEED_CONSTANT_50W : SPEED_CONSTANT_30W);
	OMEGAtoPWM = (1/SPEED_CONSTANT)*(MAX_PWM/MAX_VOLTAGE)*GEAR_RATIO;
	WHEEL_REF_LIMIT = WHEEL_REF_LIMIT_PWM/OMEGAtoPWM;
}

//Initializes the PID values
static void initPID(PIDvariables* PID, float kP, float kI, float kD) {
	//PID = PIDdefault;
	PID->kP = kP;
	PID->kI = kI;
	PID->kD = kD;

	PID->I = PIDdefault.I;
	PID->prev_e = PIDdefault.prev_e;
	PID->timeDiff = PIDdefault.timeDiff;
	PID->minOutput = PIDdefault.minOutput;
	PID->maxOutput = PIDdefault.maxOutput;
	PID->ramp = PIDdefault.ramp;
	PID->prev_PID = PIDdefault.prev_PID;
}

//clamps the input
//static float clamp(float input, float min, float max){
//	if (input<min){
//		return min;
//	} else if (input>max) {
//		return max;
//	} else {
//		return input;
//	}
//}

//limits the change in PID value
//static float ramp(float new_PID, float ramp, float prev_PID){
//	if (new_PID-prev_PID>ramp){
//		return (prev_PID+ramp);
//	} else if (new_PID-prev_PID<-ramp){
//		return (prev_PID-ramp);
//	} else {
//		return new_PID;
//	}
//}

//PID control, inline to not have multiple implementation error
inline float PID(float err, PIDvariables* K){
	float P = K->kP*err;
	K->I += err*K->timeDiff;
	float I = K->kI*K->I;
	float D = K->kD*((err-K->prev_e)/K->timeDiff);
	K->prev_e = err;
	float PIDvalue = P + I + D;
//	PIDvalue = ramp(PIDvalue, K->ramp, K->prev_PID);
//	PIDvalue = clamp(PIDvalue, K->minOutput, K->maxOutput);
	K->prev_PID = PIDvalue;
	return PIDvalue;
}

//Scales the angle to the range Pi to -Pi in radians
inline float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0){
        x += 2*M_PI;
    }
    return x - M_PI;
}

#endif /* UTILS_CONTROL_UTIL_H_ */
