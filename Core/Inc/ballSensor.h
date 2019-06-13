#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

#include "gpio_util.h"

#define NOBALL -1
#define BS_I2C_ADDR (uint16_t)(0x50 << 1)
#define MAX_DATA_SIZE 255

enum zForceStates{
	zForce_RST, // 0
	zForce_WaitForDR, // 1
	zForce_DecodeMessage, // 2
	zForce_ReadMessage, // 3
	zForce_EnableDevice, //4
	zForce_SetFreq, // 5
	zForce_SetConfig // 6
} zForceState;

typedef struct{
	int16_t x; // x coordinate, 0-700. -1 is no ball condition
	int16_t y; // y coordinate. 0-440. -1 is no ball condition
	float speed; // i am speeeeed
	uint lastSeen; // for speed calculations; current BS orientation may not be optimal for this feature
	int8_t id; // not really necessary since it is not reliable, but useful for debugging sometimes
	int8_t canKickBall; // within ?mm in y direction, set in updatePosition()
	int8_t canSeeBall; // self-explanatory, set in updatePosition()
} Position;

Position ballPosition; // system wide struct for ball position

uint8_t data[MAX_DATA_SIZE]; // byte array for received messages

extern uint8_t ball_debug; // enable ball speed print statements thru Putty ("toggle bs" command)

extern uint8_t ballSensorInitialized; // ball sensor initialization status
extern uint8_t next_message_length; // default length of next message is 2 bytes (response type and length)

/* ball sensor boot complete response */
extern uint8_t bootcomplete_response[];

/* ball sensor configuration command, to be sent before enabling device */
extern uint8_t config_command[];
extern uint8_t config_response[];

/* ball sensor frequency command */
/* set scanning frequency to 800Hz (0x3, 0x20) & idle frequency to 800Hz */
extern uint8_t freq_command[];
extern uint8_t freq_response[];

/* ball sensor enable device command */
extern uint8_t enable_command[];
extern uint8_t enable_response[];

/* ball sensor receive data notification */
extern uint8_t measurement_rx[];

/* i2c communication functions, callbacks are also implemented in .c */
void I2CTx(uint8_t tosend[], uint8_t length);
void I2CRx();

/* ballsensor functions */
void printRawData(uint8_t data[]); // prints received bytes, use for debugging (to determine new config response)
void printBallPosition(); // prints latest ball position
void parseMessage(); // decode received message and determine next FSM state
void ballSensor_Init(); // initialize
void ballSensor_DeInit(); // deinitialize
void ballSensor_Reset(); // reset
int8_t ballSensorFSM(); // FSM controller
void updatePosition(uint8_t data[]); // update position struct
void noBall(); // set no ball values for Position struct
int8_t getBallPos(); // returns latest ball position
void printBallSpeed(); // technically deprecated; to be called from main function

#endif /* BALLSENSOR_H_ */
