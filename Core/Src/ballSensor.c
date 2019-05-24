/*
 * ballSensor.c
 *
 *  Created on: Mar 30, 2018
 *      Author: Gebruiker
 */
#include "ballSensor.h"
#include "limits.h"
#include "gpio_util.h"
#include "PuTTY.h"
#include "string.h"
#include "peripheral_util.h"

int16_t y_pos_old = NOBALL;
int16_t x_pos_old = NOBALL;
uint lastSeen_old = 0;

uint error;

int8_t config = 0;
int8_t freq = 0;

enum zForceStates zForceState = zForce_RST;

uint8_t ball_debug = 0; // enable ball speed print statements

uint8_t ballSensorInitialized = 0; // ball sensor initialization routine status
uint8_t next_message_length = 2; // default length of next message is 2 bytes

/* ball sensor boot complete response */
uint8_t bootcomplete_response[] = {0xF0, 0x11, 0x40, 0x02, 0x00, 0x00, 0x63, 0x0B, 0x80, 0x01, 0x00, 0x81, 0x02, 0x03,
								   	   	  0x00, 0x82, 0x02, 0x00, 0x00};

/* ball sensor configuration command, to be sent before enabling device */
/* minX 100, minY 0, maxX 620, maxY 440, minSize 100, reported&tracked touch 1, everything else off */
uint8_t config_command[] = {0xEE, 0x40, 0xEE, 0x3E, 0x40, 0x02, 0x02, 0x00, 0x73, 0x38, 0x80, 0x01, 0x01, 0xA2,
								   0x1D, 0x80, 0x01, 0x64, 0x81, 0x01, 0x00, 0x82, 0x02, 0x02, 0x6C, 0x83, 0x02, 0x01,
								   0xB8, 0x84, 0x01, 0x00, 0x85, 0x01, 0x00, 0x86, 0x01, 0x00, 0x87, 0x01, 0x00, 0x88,
								   0x01, 0x00, 0xA4, 0x09, 0x80, 0x01, 0x00, 0x82, 0x01, 0xFF, 0x83, 0x01, 0x64, 0x86,
								   0x01, 0x01, 0xA7, 0x06, 0x80, 0x01, 0x00, 0x81, 0x01, 0x00};
uint8_t config_response[] = {0xEF, 0x43, 0x40, 0x02, 0x02, 0x00, 0x73, 0x3D, 0xA2, 0x1D, 0x80, 0x01, 0x64, 0x81,
									0x01, 0x00, 0x82, 0x02, 0x02, 0x6C, 0x83, 0x02, 0x01, 0xB8, 0x84, 0x01, 0x00, 0x85,
									0x01, 0x00, 0x86, 0x01, 0x00, 0x87, 0x01, 0x00, 0x88, 0x01, 0x00, 0xA4, 0x0C, 0x80,
									0x01, 0x00, 0x81, 0x01, 0x00, 0x82, 0x01, 0xFF, 0x83, 0x01, 0x64, 0x85, 0x01, 0x00,
									0x86, 0x01, 0x01, 0xA7, 0x08, 0x80, 0x02, 0x02, 0x08, 0x81, 0x02, 0x01, 0xB8};

/* ball sensor frequency command */
/* set scanning frequency to 800Hz (0x3, 0x20) & idle frequency to 800Hz */
uint8_t freq_command[] = {0xEE, 0x10, 0xEE, 0x0E, 0x40, 0x02, 0x02, 0x00, 0x68, 0x08, 0x80, 0x02, 0x03, 0x20, 0x82, 0x02, 0x03, 0x20};
uint8_t freq_response[] = {0xEF, 0x0E, 0x40, 0x02, 0x00, 0x00, 0x68, 0x08, 0x80, 0x02, 0x03, 0x20, 0x82, 0x02, 0x03, 0x20};

/* ball sensor enable device command */
uint8_t enable_command[] = 	{0xEE, 0x0B, 0xEE, 0x09, 0x40, 0x02, 0x02, 0x00, 0x65, 0x03, 0x81, 0x01, 0x00};
uint8_t enable_response[] = {0xEF, 0x09, 0x40, 0x02, 0x02, 0x00, 0x65, 0x03, 0x81, 0x01, 0x00};


/* ball sensor receive data notification */
uint8_t measurement_rx[] = {0xF0, 0x11, 0x40, 0x02, 0x02};

void ballSensorInit()
{
	//HAL_I2C_Init(&hi2c1);
	//resetKickChipData();
	noBall();
	//Putty_printf ("init ballsensor\n\r");
	ballSensorReset();

	next_message_length = 2;
	set_Pin(BS_RST_pin, 1);
	int currentTime = HAL_GetTick();
	zForceState = zForce_WaitForDR;
	while(  !read_Pin(BS_IRQ_pin)  &&  (HAL_GetTick()-currentTime < 1000)  );
	I2CRx();
}

void ballSensorReset() {
	//Putty_printf ("BS_RST\n\r");
	ballSensorInitialized = 0;
	noBall();
	set_Pin(BS_RST_pin, 0);
	//Putty_printf("going to waitfordr\n\r");
	zForceState = zForce_WaitForDR;
}

void printRawData(uint8_t data[]) {
    Putty_printf("\n\rdata = [");
    for(uint i = 0; i < next_message_length; i++){
      Putty_printf("0x%02X, ", data[i]);
    }
    Putty_printf("]\n\r");
}

void printBallPosition() {
	//Putty_printf("BALLSENSOR - x:\t %lu \t y:\t %lu \n\r", ballPosition.x, ballPosition.y);
	Putty_printf("\n\rBALLS\n\rx,y: %d  %d\n\r", ballPosition.x, ballPosition.y);
	Putty_printf("speed %02f and timediff %d\n\r", ballPosition.speed, ballPosition.lastSeen-lastSeen_old);
	Putty_printf("id is %d\n\r", ballPosition.id);
	Putty_printf("canKickBall: %d, canSeeBall: %d\n\r", ballPosition.canKickBall, ballPosition.canSeeBall);
	//Putty_printf("size %hu, %hu\n\r", data[16], data[17]);
}

void updatePosition(uint8_t data[]) {
  uint16_t x;
  x = data[12] << 8;
  x |= data[13];
  uint16_t y;
  y = data[14] << 8;
  y |= data[15];
  //Putty_printf("BALLSENSOR - x:\t %d \t y:\t %d \n\r", x,y);

  ballPosition.x = x;
  ballPosition.y = y;  
  ballPosition.lastSeen = HAL_GetTick();
  ballPosition.id = data[10];
  ballPosition.canKickBall = (y<250) ? 1 : 0;
  ballPosition.canSeeBall = 1;
	//printBallPosition();
}

void noBall() {
	ballPosition.x = ballPosition.y = y_pos_old = x_pos_old = NOBALL;
	ballPosition.speed = 0;
	ballPosition.lastSeen = 0;
	ballPosition.id = -1;
	ballPosition.canKickBall = 0;
	ballPosition.canSeeBall = 0;
	lastSeen_old = 0;
}

void parseMessage() {
	if(!memcmp( data, bootcomplete_response, 10) && !memcmp( data+11, bootcomplete_response+11, 3) && !memcmp( data+13, bootcomplete_response+13, 3)) {
	  //Putty_printf("BS_BOOT\n\r");
	  zForceState = zForce_SetConfig;
	}

	else if (!memcmp(data, config_response, sizeof(config_response))) {
		//Putty_printf("BS_CONF\n\r");
		zForceState = zForce_SetFreq;
	}

    else if(!memcmp(data, freq_response, sizeof(freq_response))) {
        //Putty_printf("BS_FREQ\r\n");
        zForceState = zForce_EnableDevice;
    }

	else if (!memcmp(data, enable_response, sizeof(enable_response))) {
        ballSensorInitialized = 1;
        Putty_printf("BS_INIT\n\r");
        zForceState = zForce_WaitForDR;
	}

	else if(!memcmp(data,measurement_rx, sizeof(measurement_rx)) && data[11] != 2) {
		// ball detected if event ID (data[11]) is 0 or 1. data[11]==2 is for lost objects
		float speed_new = 0;
        zForceState = zForce_WaitForDR;

        if (ballPosition.id == -1) {
        	// after NOBALL need to update with base information to avoid wrong (off) calculations
        	updatePosition(data);
        	y_pos_old = ballPosition.y;
        	lastSeen_old = ballPosition.lastSeen;
        	ballPosition.id = data[10];
        }

        else { // continuous ball detection
        	x_pos_old = ballPosition.x; // store old coordinates
        	y_pos_old = ballPosition.y;
//        	updatePosition(data); // update struct
//        }
//        else { // disable this else block to only report canSeeBall and canKickBall
    		lastSeen_old = ballPosition.lastSeen;
    		updatePosition(data);
			// timer overflow, adjust time difference by UINT_MAX+1
			// coordinates are in millimeters AFTER div by 10, timestamp in milliseconds
			if (lastSeen_old >= ballPosition.lastSeen) {
				speed_new = ((float) (ballPosition.y - (float) (y_pos_old))) / (10*(ballPosition.lastSeen+UINT_MAX+1 - lastSeen_old));
			}
			else {
				speed_new = ((float) (ballPosition.y - (float) (y_pos_old))) / (10*(ballPosition.lastSeen - lastSeen_old));
			}

			ballPosition.speed = speed_new;
			if (ballPosition.speed != 0 && ball_debug == 1) Putty_printf ("%d, %d, %.3f\n\r", ballPosition.y, y_pos_old, ballPosition.speed);
       }
	}

	else { //ignore any other data
	  noBall();
	  /* if need to print config or freq response data, add things below to zForceState statement
	   * (config) ? zForce_SetFreq : OR (freq) ? zForce_EnableDevice :
	   * and enable printRawData in the next state
	   */
	  zForceState = zForce_WaitForDR;
	}

}

void printBallSpeed() {
	if (ballPosition.speed != 0)
		Putty_printf ("%d, %d, %d, %d, %.3f\n\r", ballPosition.y, y_pos_old, ballPosition.lastSeen, lastSeen_old, ballPosition.speed);
}

int8_t ballSensorFSM()
{
	//Putty_printf("HAL_I2C_GetState = [%02x]\n\r", HAL_I2C_GetState(&hi2c1));
	if(HAL_I2C_GetState(BS_I2C) != HAL_I2C_STATE_READY) {
		//Putty_printf("!=i2cstateready\n\r");
		return getBallPos();
	}

	//Putty_printf("zForceState = [%d]\n\r", zForceState);
	switch(zForceState){
	case zForce_RST:// device to be kept in reset
		Putty_printf("zForce_RST\n\r");
		ballSensorReset();
		break;

	case zForce_SetConfig:
		//Putty_printf("zForce_SetConfig\n\r");
		config = 1;
		I2CTx(config_command, sizeof(config_command));
		break;

	case zForce_SetFreq:
		//if (config) printRawData(data);
		config = 0;
		freq = 1;
		//Putty_printf("zForce_SetFreq\n\r");
		I2CTx(freq_command, sizeof(freq_command));
		break;

	case zForce_EnableDevice:
		//if (freq) printRawData(data);
		freq = 0;
		//Putty_printf("zForce_EnableDevice\n\r");
		I2CTx(enable_command, sizeof(enable_command));
		break;

	case zForce_WaitForDR:// when DR(Data Ready) is high, message length needs to be read
		//Putty_printf("zForce_WaitForDR\n\r");
		next_message_length = 2;
		set_Pin(BS_RST_pin, 1);
		if(read_Pin(BS_IRQ_pin)){
			//Putty_printf("data ready\n\r");
			I2CRx();
		}
        break;

	case zForce_DecodeMessage:// message is received and needs to be decoded
		//Putty_printf("zForce_DecodeMessage\n\r");
		next_message_length = data[1]; // double check this message length with the default meas_rx response length for comparison
		zForceState = zForce_ReadMessage;
		break;

	case zForce_ReadMessage:// when message length is known it should be received
		//Putty_printf("zForce_ReadMessage\n\r");
		if(read_Pin(BS_IRQ_pin)){
			I2CRx();
		}
		break;
	}

    //PuttyInterface_Update(&puttystruct);
	//Putty_printf("ball: %i\n",getBallPos());
	//Putty_printf("ball: %i\n",getBallPos());
	return getBallPos();
}

int8_t getBallPos() {
	if(ballPosition.x != NOBALL) {
		return ballPosition.x/10;
	}
	return NOBALL;
}

void I2CTx(uint8_t tosend[], uint8_t length) {
    if(HAL_OK != (error = HAL_I2C_Master_Transmit_IT(BS_I2C, BS_I2C_ADDR, tosend, length))){// in case of error; put the device in reset
  	  set_Pin(BS_RST_pin, 0);
        Putty_printf("BALLSENSOR - i2c transmit failed with error [%d]!\n\rzForce stopped\n\r", error);
        zForceState = zForce_RST;
    }
//    ballSensorFSM(); // call to FSM to keep it rolling
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
	zForceState = zForce_WaitForDR;
//	ballSensorFSM(); // call to FSM to keep it rolling
}

void I2CRx() {
	if(HAL_OK != (error = HAL_I2C_Master_Receive_IT(BS_I2C, BS_I2C_ADDR, data, next_message_length))){
		Putty_printf("BALLSENSOR - i2c read failed with error [%d]!\n\rzForce stopped\n\r", error);
		set_Pin(BS_RST_pin, 0);
		zForceState = zForce_RST;
	}
	//Putty_printf("rx ballsensor\n\r");
//	ballSensorFSM(); // call to FSM to keep it rolling
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *I2cHandle)
{
//	Putty_printf("ballsensor rx callback\n\r");
	if(zForceState == zForce_WaitForDR) {
		 zForceState = zForce_DecodeMessage;
	}
	else if(zForceState == zForce_ReadMessage) {
		parseMessage();
	}
//	ballSensorFSM(); // call to FSM to keep it rolling
}
