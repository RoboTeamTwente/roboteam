/*
 * FT812Q.h
 *
 *  Created on: May 16, 2019
 *      Author: selina
 */

#ifndef FT812Q_FT812Q_H_
#define FT812Q_FT812Q_H_

#include "main.h"
#include "stm32f7xx_hal_def.h"
#include "stm32f7xx_hal_qspi.h"
#include "FT812Q_Constants.h"
#include <stdbool.h>

extern QSPI_HandleTypeDef hqspi;

/* BASIC */
void 		display_Init		();
void		display_Deinit		();
uint32_t 	writeDispBuf		(uint32_t base, uint32_t size, uint8_t* data);
void 		writeDisplay		(uint32_t address, uint32_t size, uint8_t* data);
uint8_t*	readDisplay			(uint32_t address, uint32_t size, uint8_t* data);

/* TOUCH */
void	 	readTouch			(uint16_t* touchPoint);
uint8_t		isInArea			(uint16_t* point);

/* FUNCTIONS FOR DRAWING */
uint8_t* 	CLEAR				(uint8_t c, uint8_t s, uint8_t t); // Clears screen
uint8_t* 	CLEAR_COLOR_RGB		(uint8_t red, uint8_t green, uint8_t blue); // Set background color
uint8_t* 	COLOR_RGB			(uint8_t red, uint8_t green, uint8_t blue); // Sets drawing color
uint8_t* 	POINT_SIZE			(uint16_t size); // Set the point size
uint8_t* 	LINE_WIDTH			(uint16_t width); // Set the line width
uint8_t* 	VERTEX_FORMAT		(uint8_t size);

uint8_t* 	POINT_DATA			(uint16_t x, uint16_t y); // Construct data for a point at (x,y)
uint8_t* 	LETTER_DATA			(uint16_t x, uint16_t y, uint8_t font, uint8_t letter); // Construct data for a letter at (x,y)

/* STATE MACHINE */
typedef enum _DISPLAY_STATES {
	DISPLAY_STATE_DEINITIALIZED,
	DISPLAY_STATE_INITIALIZED,
    DISPLAY_STATE_MAIN,
    DISPLAY_STATE_ROBOT,
} DISPLAY_STATES;

typedef enum _TOUCH_STATES {
	TOUCH_STATE_RELEASED,
	TOUCH_STATE_PRESSED
} TOUCH_STATES;

typedef struct _TouchState {
	TOUCH_STATES state;
	uint16_t x;
	uint8_t y;
	bool handled;
} TouchState;

#endif /* FT812Q_FT812Q_H_ */
