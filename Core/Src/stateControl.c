
#include "../Inc/stateControl.h"

///////////////////////////////////////////////////// VARIABLES

static PID_states status = off;
static PIDvariables stateK[3];
static float stateRef[3] = {0.0f};
static float wheelRef[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static float state[3] = {0.0f};

///////////////////////////////////////////////////// PRIVATE FUNCTION DECLARATIONS

//Transforms body velocity to wheel speed
static void body2Wheels(float wheelSpeed[4], float state[3]);

//Transfer global coordinate frame to local coordinate frame
static void global2Local(float input[3], float output[2], float yaw);

//Determine the desired wheel speed given the desired velocities
static void translationVelControl(float state[3], float velRef[3], float translationalRef[4]);

//Determine the desired wheel speed given the desired angle
static float angleControl(float angleRef, float angle);

///////////////////////////////////////////////////// PUBLIC FUNCTION IMPLEMENTATIONS

int stateControl_Init(){
	status = on;
	initPID(&stateK[body_x], 0.1, 0.0, 0.0);
	initPID(&stateK[body_y], 0.4, 0.0, 0.0);
	initPID(&stateK[body_w], 20.0, 1.5, 0.0);
	HAL_TIM_Base_Start_IT(TIM_CONTROL);
	return 0;
}

int stateControl_DeInit(){
	status = off;
	HAL_TIM_Base_Stop_IT(TIM_CONTROL);
	return 0;
}

void stateControl_Update(){
	if (status == on){
		float translationalRef[4] = {0.0f};
		translationVelControl(state, stateRef, translationalRef);

		float angularRef = angleControl(stateRef[body_w], state[body_w]);

		for (wheel_names wheel=wheels_RF; wheel<=wheels_LF; wheel++){
			wheelRef[wheel] = translationalRef[wheel] + angularRef;
		}
	}
}

void stateControl_SetRef(float input[3]){
	stateRef[body_x] = input[body_x];
	stateRef[body_y] = input[body_y];
	stateRef[body_w] = input[body_w];
}

float* stateControl_GetWheelRef() {
	return wheelRef;
}

void stateControl_SetState(float input[3]){
	state[body_x] = input[body_x];
	state[body_y] = input[body_y];
	state[body_w] = input[body_w];
}

///////////////////////////////////////////////////// PRIVATE FUNCTION IMPLEMENTATIONS

static void body2Wheels(float wheelSpeed[4], float vel[3]){
	//mixing matrix
	float velx2wheel = (vel[body_x]*sin60/rad_wheel);
	float vely2wheel = (vel[body_y]*cos60/rad_wheel);
	//float rot2wheel =  (rad_robot*vel[body_w]/rad_wheel);
	wheelSpeed[wheels_RF] = (velx2wheel + vely2wheel);
	wheelSpeed[wheels_RB] = (velx2wheel - vely2wheel);
	wheelSpeed[wheels_LB] = (-velx2wheel - vely2wheel);
	wheelSpeed[wheels_LF] = (-velx2wheel + vely2wheel);
}

static void global2Local(float global[3], float local[3], float  yaw){
	//trigonometry
	local[body_x] = cosf(yaw)*global[body_x]+sinf(yaw)*global[body_y];
	local[body_y] = -sinf(yaw)*global[body_x]+cosf(yaw)*global[body_y];
	local[body_w] = global[body_w];
}

static void translationVelControl(float state[3], float stateRef[3], float translationalRef[4]){
	float stateLocalRef[3] = {0, 0, 0};
	global2Local(stateRef, stateLocalRef, state[body_w]); //transfer global to local

	// Manually adjusting velocity command
	//     Explanation: see Velocity Difference file on drive (https://docs.google.com/document/d/1pGKysiwpu19DKLpAZ4GpluMV7UBhBQZ65YMTtI7bd_8/)
	stateLocalRef[body_x] = 1.063 * stateLocalRef[body_x];
	stateLocalRef[body_y] = 1.308 * stateLocalRef[body_y];

	// Local control
	float velxErr = (stateLocalRef[body_x] - state[body_x]);
	float velyErr = (stateLocalRef[body_y] - state[body_y]);
	stateLocalRef[body_x] += PID(velxErr, &stateK[body_x]);
	stateLocalRef[body_y] += PID(velyErr, &stateK[body_y]);

	body2Wheels(translationalRef, stateLocalRef); //translate velocity to wheel speed
}

static float angleControl(float angleRef, float angle){
	float angleErr = constrainAngle(angleRef - angle);//constrain it to one circle turn
	if (fabs(angleErr) < YAW_MARGIN) { // reset the I to zero everytime the target has been reached
		angleErr = 0;
		stateK[body_w].I = 0;
	}
	return PID(angleErr, &stateK[body_w]);// PID control from control_util.h
}




