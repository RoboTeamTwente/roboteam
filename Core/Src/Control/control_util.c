#include "main.h"
#include "control_util.h"

void control_util_Init() {
	bool MOTORS_50W = true; // Keep this on the offchance that we're going to use the 30W motors again
	MAX_VOLTAGE = MOTORS_50W ? MAX_VOLTAGE_50W : MAX_VOLTAGE_30W;
	SPEED_CONSTANT = 2*M_PI/60.0 * (MOTORS_50W ? SPEED_CONSTANT_50W : SPEED_CONSTANT_30W);
	OMEGAtoPWM = (1/SPEED_CONSTANT)*(MAX_PWM/MAX_VOLTAGE)*GEAR_RATIO;
	WHEEL_REF_LIMIT = WHEEL_REF_LIMIT_PWM/OMEGAtoPWM;
}

void initPID(PIDvariables* PID, float kP, float kI, float kD) {
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

float PID(float err, PIDvariables* K){
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

float constrainAngle(float x){
    x = fmodf(x + M_PI, 2*M_PI);
    if (x < 0){
        x += 2*M_PI;
    }
    return x - M_PI;
}