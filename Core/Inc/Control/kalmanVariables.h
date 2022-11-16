
/* Description: Kalman filter variables
 *
 * Instructions:
 * 1) Initializes the necessary arrays and variables for the Kalman filter
 *
 * Extra functions:
 *
 * Notes:
 *
*/

#ifndef KALMAN_KALMANVARIABLES_H_
#define KALMAN_KALMANVARIABLES_H_
///////////////////////////////////////////////////// DEFINITIONS

#define STATE 4
#define OBSERVE 4
#define TIMESTEP 0.01

// certainties
#define VEL_VAR 0.000025F // variance in the velocity measurements
#define ACC_VAR 6.25F // variance in the acceleration measurements
#define STATE_VAR 0.5F // variance in the predicted state
#define RAND_VAR 0.5F // variance in the random force

///////////////////////////////////////////////////// VARIABLES
// TODO: Ask old team members to explain the variables and their use.

//create arrays
float aF[STATE*STATE] = {
		1, TIMESTEP, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, TIMESTEP,
		0, 0, 0, 1};
	
float aH[OBSERVE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};
	
float aR[OBSERVE*OBSERVE] = {
		VEL_VAR, 0, 0, 0,
		0, ACC_VAR, 0, 0,
		0, 0, VEL_VAR, 0,
		0, 0, 0, ACC_VAR};

float aI[STATE*STATE] = {
		1,0,0,0,
		0,1,0,0,
		0,0,1,0,
		0,0,0,1};

float aPold[STATE*STATE] = {
		STATE_VAR,0,0,0,
		0,STATE_VAR,0,0,
		0,0,STATE_VAR,0,
		0,0,0,STATE_VAR};
		
float aB[STATE*STATE] = {
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1};
	
float aQ[STATE*STATE] = {
		TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR, 0, 0,
		TIMESTEP*RAND_VAR, RAND_VAR, 0, 0,
		0, 0, TIMESTEP*TIMESTEP*RAND_VAR, TIMESTEP*RAND_VAR,
		0, 0, TIMESTEP*RAND_VAR, RAND_VAR};

//empty arrays
float aXold[STATE] = {0.0f};			// The old state
float az[OBSERVE] = {0.0f};				
float aXcurrent[STATE] = {0.0f};		// The current tate
float aFX[STATE] = {0.0f};				// The state transition from this to the next state
float aFt[STATE*STATE] = {0.0f};
float aFP[STATE*STATE] = {0.0f};
float aPcurrent[STATE*STATE] = {0.0f};	// The current prediction matrix
float ayold[OBSERVE] = {0.0f};			
float aHX[OBSERVE] = {0.0f};
float aS[OBSERVE*OBSERVE] = {0.0f};
float aHt[STATE*OBSERVE] = {0.0f};
float aPHt[STATE*OBSERVE] = {0.0f};
float aFPFt[STATE*STATE] = {0.0f};
float aHPHt[OBSERVE*OBSERVE] = {0.0f};
float aK[STATE*OBSERVE] = {0.0f};		// Kalman gain
float aSi[OBSERVE*OBSERVE] = {0.0f};
float aXnew[STATE] = {0.0f};			// The next state
float aKy[STATE] = {0.0f};
float aPnew[STATE*STATE] = {0.0f};		// The next prediction matrix
float aKt[OBSERVE*STATE] = {0.0f};
float aKR[STATE*OBSERVE] = {0.0f};
float aKRKt[STATE*STATE] = {0.0f};
float aKH[STATE*STATE] = {0.0f};
float aI_KH[STATE*STATE] = {0.0f};
float aI_KHt[STATE*STATE] = {0.0f};
float aI_KHP[STATE*STATE] = {0.0f};
float aI_KHPI_KHt[STATE*STATE] = {0.0f};
float aU[STATE] = {0.0f};
float aBU[STATE] = {0.0f};

#endif /* KALMAN_KALMANV_H_ */
