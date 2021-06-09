#ifndef BALLSENSOR_H_
#define BALLSENSOR_H_

#include "gpio_util.h"

#define BALLSENSOR_NO_BALL -1
#define BS_I2C_ADDR (uint16_t)(0x50 << 1)

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


/* ballsensor functions */
bool ballSensor_Init(); // initialize
void ballSensor_DeInit(); // deinitialize
void ballSensor_Reset(); // reset
void ballSensor_IRQ_Handler(); // irq handler
bool ballSensor_isInitialized();
bool ballSensor_I2C_Rx(); // blocking receive for initialization (receives 2bytes and then full message)


/* ballsensor private functions */
void bs_I2C_error(uint8_t error);

bool bs_Boot(); // receive bootcomplete response

bool bs_setDeviceConfiguration(); // send configuration message
bool bs_checkDeviceConfiguration(); // check configuration response

bool bs_getTouchFormat();
bool bs_checkTouchFormat();

bool bs_setScanningFrequency();
bool bs_checkScanningFrequency();

bool bs_setEnableDevice(); // send enable device message
bool bs_checkEnableDevice(); // check enable device response

bool bs_CheckMeasurement(); // check received measurement notification

void bs_updatePosition(uint8_t data[]); // update position struct
void bs_NoBall(); // set no ball values for position struct


/* Page 83
* 87654321 = byte
* ccsttttt = usage. 
* c = class tag. 00 = unused; 01 = APPLICATION; 10 = Context-Specific; 11 = PRIVATE;
* s = indication either list or variable; 0 = PRIMITIVE (variable); 1 = SEQUENCE (list);
* t = tag number. Simply a 5-bit number, between 0 and 31.
*/

#define BS_REQUEST 0xEE /* = 11101110 = 11 1 01110 = PRIVATE SEQUENCE 14 = [PRIVATE 14] = request */
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
		#define BS_NUMBER_OF_TRACKED_TOUCHES 0x80
		#define BS_NUMBER_OF_REPORTED_TOUCHES 0x86
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
		#define BS_BOOT_COMPLETE_NOTIFICATION 0x63
			#define BS_ASIC_STATUS 0x80
			#define BS_RESET_SOURCE 0x81

#endif /* BALLSENSOR_H_ */
