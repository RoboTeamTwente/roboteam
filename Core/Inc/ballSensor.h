#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

#include "gpio_util.h"

#define BALLSENSOR_NO_BALL -1
#define BS_I2C_ADDR (uint16_t)(0x50 << 1)
#define MAX_DATA_SIZE 255

/* position struct */
volatile typedef struct{
	int16_t x; // x coordinate, 0-700. -1 is no ball condition
	int16_t y; // y coordinate. 0-440. -1 is no ball condition
	float speed; // i am speeeeed, not used cuz orientation
	uint32_t lastSeen; // for speed calculations; current BS orientation may not be optimal for this feature
	int8_t id; // not really necessary since it is not reliable, but useful for debugging sometimes
	int8_t canKickBall; // within ?mm in y direction, set in updatePosition()
	int8_t canSeeBall; // self-explanatory, set in updatePosition()
} Position;

Position ballPosition; // system wide struct for ball position

uint8_t data[MAX_DATA_SIZE] __attribute__((aligned(16))); // byte array for received messages

extern uint8_t ball_debug; // enable ball speed print statements thru Putty ("toggle bs" command)

extern volatile uint8_t ballSensorInitialized; // ball sensor initialization status
extern uint8_t next_message_length; // default length of next message is 2 bytes (response type and length)

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
bool ballSensor_I2C_Rx(); // blocking receive for initialization (receives 2bytes and then full message)

/* ballsensor functions */
void printRawData(uint8_t data[]); // prints received bytes, use for debugging (to determine new config response)
void printBallPosition(); // prints latest ball position
bool ballSensor_Init(); // initialize
void ballSensor_DeInit(); // deinitialize
void ballSensor_Reset(); // reset
void ballSensor_IRQ_Handler(); // irq handler
int8_t ballSensorFSM(); // FSM controller
void updatePosition(uint8_t data[]); // update position struct
void ballSensor_NoBall(); // set no ball values for Position struct
bool ballSensor_isWorking();

/* print functions */
void printRawData(uint8_t data[]); // prints received bytes, use for debugging (e.g. to determine new config response)
void printBallPosition(); // prints latest ball position

/* ballsensor private functions */
bool bs_Boot(); // receive bootcomplete response
bool bs_SetConfig(); // send configuration message
bool bs_CheckConfig(); // check configuration response
bool bs_requestTouchFormat();
bool bs_readTouchFormat();
bool bs_SetFreq(); // send frequency settings
bool bs_CheckFreq(); // check frequency settings
bool bs_EnableDevice(); // send enable device message
bool bs_CheckEnable(); // check enable device response
void bs_CheckMeasurement(); // check received measurement notification
void updatePosition(uint8_t data[]); // update position struct
void ballSensor_NoBall(); // set no ball values for position struct


/* Page 83
* 87654321 = byte
* ccsttttt = usage. 
* c = class tag. 00 = unused; 01 = APPLICATION; 10 = Context-Specific; 11 = PRIVATE;
* s = indication either list or variable; 0 = PRIMITIVE (variable); 1 = SEQUENCE (list);
* t = tag number. Simply a 5-bit number, between 0 and 31.
*/

#define BS_REQUEST 0xEE /* = 11101110 = 11 1 01110 = PRIVATE SEQUENCE 14 = [PRIVATE 14] = request*/
#define BS_RESPONSE 0xEF
	#define BS_DEVICE_ADDRESS_AIR 0x40, 0x02, 0x02, 0x00
	#define BS_DEVICE_ADDRESS_PLATFORM 0x40, 0x02, 0x00, 0x00
	#define BS_ENABLE_SEQUENCE 0x65
		#define BS_ENABLE 0x81
			#define BS_ENABLE_CONTINUOUS 0x00
	#define BS_TOUCH_FORMAT 0x66 /* touchFormat [APPLICATION 6] SEQUENCE = 01 1 00110 = 0x66 */
	#define BS_FREQUENCY 0x68
		#define BS_FINGER 0x80
		#define BS_IDLE 0x82
	#define BS_DEVICE_CONFIGURATION 0x73
		#define BS_NUMBEROFTRACKEDTOUCHES 0x80
		#define BS_SUBTOUCHACTIVEAREA 0xA2
			#define BS_LOWBOUNDX 0x80
			#define BS_LOWBOUNDY 0x81
			#define BS_HIGHBOUNDX 0x82
			#define BS_HIGHBOUNDY 0x83
			#define BS_REVERSEX 0x84
			#define BS_REVERSEY 0x85
			#define BS_FLIPXY 0x86
			#define BS_OFFSETX 0x87
			#define BS_OFFSETY 0x88
		#define BS_SIZERESTRICTION 0xA4
			#define BS_MAXSIZEENABLED 0x80
			#define BS_MAXSIZE 0x81
			#define BS_MINSIZEENABLED 0x82
			#define BS_MINSIZE 0x83
#define BS_NOTIFICATION 0xF0
	#define BS_TOUCH_NOTIFICATION_SEQUENCE 0xA0
		#define BS_TOUCH_NOTIFICATION 0x42
/*
11 1 10000
F0 11 
	40 02 02 00 
	A0 0B // A0 10 1 00000 => Context-specific Sequence 0 : 
		42 09 // 42 01 0 00010 [APPLICATION 2] PRIMITIVE (OCTET STRING) TouchNotification
		0E 01 00 17 00 7B 00 41 00
	 // 10 11 12 13 14 15 16 17 18
F0 11 
	40 02 02 00 
		A0 0B 42 09 0E 02 00 00 00 79 00 41 00 BC


EF 0C RESPONSE
	40 02 02 00 DEVICE_ADDRESS
	66 06 : 01 1 00110 [APPLICATION 6] touchFormat
		41 04 : 01 0 00001 = [APPLICATION 1] BIT STRING TouchDescriptor
			01 F6 18 04 	0000 0001    1111 0110    0001 1000    0000 0100

*/




#endif /* BALLSENSOR_H_ */
