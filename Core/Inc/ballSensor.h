#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

#include "gpio_util.h"

#define NOBALL -1
#define BS_I2C_ADDR (uint16_t)(0x50 << 1)
#define MAX_DATA_SIZE 255

/* position struct */
typedef struct{
	int16_t x; // x coordinate, 0-700. -1 is no ball condition
	int16_t y; // y coordinate. 0-440. -1 is no ball condition
	float speed; // i am speeeeed, not used cuz orientation
	uint lastSeen; // for speed calculations; current BS orientation may not be optimal for this feature
	int8_t id; // not really necessary since it is not reliable, but useful for debugging sometimes
	int8_t canKickBall; // within ?mm in y direction, set in updatePosition()
	int8_t canSeeBall; // self-explanatory, set in updatePosition()
} Position;

Position ballPosition; // system wide struct for ball position

uint8_t data[MAX_DATA_SIZE] __attribute__((aligned(16))); // byte array for received messages

extern uint8_t ball_debug; // enable ball speed print statements thru Putty ("toggle bs" command)

extern uint8_t ballSensorInitialized; // ball sensor initialization status
extern uint8_t next_message_length; // default length of next message is 2 bytes (response type and length)

extern bool bs_DMA_INUSE;
extern uint8_t init_attempts;

/* ball sensor boot complete response */
extern uint8_t bootcomplete_response[];

/* ball sensor configuration command, to be sent before enabling device */
extern uint8_t config_command[];
extern uint8_t config_response[];

/* ball sensor frequency command */
extern uint8_t freq_command[];
extern uint8_t freq_response[];

/* ball sensor enable device command */
extern uint8_t enable_command[];
extern uint8_t enable_response[];

/* ball sensor receive data notification */
extern uint8_t measurement_rx[];

/* i2c communication functions, callbacks are also implemented in .c */
void I2CTx_IT(uint8_t tosend[], uint8_t length); // transmit in interrupt mode; not used
void I2C_Rx_DMA(); // non-blocking DMA receive for measurement messages in IRQ handler
bool I2C_Rx(); // blocking receive for initialization (receives 2bytes and then full message)

/* ballsensor public functions */
bool ballSensorInit(); // initialize
void ballSensorDeInit(); // de-initialize (keep under reset)
void ballSensorReset(); // reset
void ballSensor_IRQ_Handler(); // irq handler
int8_t getBallPos(); // returns latest ball position

/* print functions */
void printRawData(uint8_t data[]); // prints received bytes, use for debugging (e.g. to determine new config response)
void printBallPosition(); // prints latest ball position

/* ballsensor private functions */
bool bs_Boot(); // receive bootcomplete response
bool bs_SetConfig(); // send configuration message
bool bs_CheckConfig(); // check configuration response
bool bs_SetFreq(); // send frequency settings
bool bs_CheckFreq(); // check frequency settings
bool bs_EnableDevice(); // send enable device message
bool bs_CheckEnable(); // check enable device response
void bs_CheckMeasurement(); // check received measurement notification
void updatePosition(uint8_t data[]); // update position struct
void noBall(); // set no ball values for position struct

#endif /* BALLSENSOR_H_ */
