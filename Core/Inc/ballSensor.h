/*
 * ballSensor.h
 *
 *  Created on: Mar 27, 2018
 *      Author: Gebruiker
 */

#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

//#include "i2c.h"
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
	int16_t x;
	int16_t y;
	float speed;
	uint lastSeen;
	int8_t id; // not really necessary since it is not reliable, but useful for debugging sometimes
	int8_t canKickBall; // within 25mm in Y direction
	int8_t canSeeBall; // self-explanatory
} Position;

Position ballPosition;

uint8_t data[MAX_DATA_SIZE]; // byte array for received messages

extern uint8_t ball_debug; // enable ball speed print statements

extern uint8_t ballSensorInitialized; // ball sensor initialization routine status
extern uint8_t next_message_length; // default length of next message is 2 bytes

/* ball sensor boot complete response */
extern uint8_t bootcomplete_response[];

/* ball sensor configuration command, to be sent before enabling device */
/* minX 100, minY 0, maxX 620, maxY 440, minSize 100, reported&tracked touch 1, everything else off */
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

void I2CTx(uint8_t tosend[], uint8_t length);
void I2CRx();

void printRawData(uint8_t data[]);
void printBallPosition();
void parseMessage();
void ballSensorInit();
void ballSensorReset();
int8_t ballSensorFSM();

void noBall();
int8_t getBallPos();
void printBallSpeed(); // to be called from main function
#endif /* BALLSENSOR_H_ */
