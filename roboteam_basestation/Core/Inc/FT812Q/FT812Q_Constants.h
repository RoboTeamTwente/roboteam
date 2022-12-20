/*
 * FT812Q_Constants.h
 *
 *  Created on: May 16, 2019
 *      Author: selina
 */

#ifndef FT812Q_FT812Q_CONSTANTS_H_
#define FT812Q_FT812Q_CONSTANTS_H_

#include <inttypes.h>

/* HOST COMMANDS */
#define DISPLAY_SET_ACTIVE 				0x000000
#define DISPLAY_GET_ACTIVE				0x302000
#define DISPLAY_SET_STANDBY				0x000000
#define DISPLAY_SET_SLEEP				0x000000
#define DISPLAY_SET_POWERDOWN			0x000000

/* REGISTERS */
#define REG_PWM_HZ 						0x3020D0
#define REG_PWM_DUTY					0x3020D4
#define REG_GPIO						0x302094
#define REG_GPIO_DIR					0x302090
#define REG_HCYCLE						0x30202C
#define REG_HOFFSET						0x302030
#define REG_HSYNC0						0x302038
#define REG_HSYNC1						0x30203C
#define REG_VCYCLE						0x302040
#define REG_VOFFSET						0x302044
#define	REG_VSYNC0						0x30204C
#define	REG_VSYNC1						0x302050
#define	REG_SWIZZLE						0x302064
#define	REG_PCLK_POL					0x30206C
#define	REG_CSPREAD						0x302068
#define	REG_HSIZE						0x302034
#define	REG_VSIZE						0x302048
#define REG_DLSWAP						0x302054
#define REG_PCLK						0x302070
#define REG_DITHER						0x302060
#define REG_TAG_X						0x302074
#define REG_TOUCH_RAW_XY				0x30211C
#define REG_TOUCH_CONFIG 				0x302168
#define REG_TOUCH_MODE					0x302104
#define REG_TOUCH_SCREEN_XY				0x302124

/* DISPLAY LIST WRITE */
#define RAM_DL							0x300000
#define RAM_CMD							0x308000

/* FONT */
#define ROM_FONTROOT					0x2FFFFC

/* DATA */
/* BUFFER */
extern uint8_t* DispBuf;				// pointer to display buffer
#define DISP_BUF_LENGTH 				4096

/* DISPLAY RESOLUTION */
extern uint16_t XRES;
extern uint16_t YRES;

/* POWER MODES */
extern uint8_t POWERDOWN[];
extern uint8_t SLEEP[];
extern uint8_t STANDBY[];
extern uint8_t ACTIVE[];

/* DISPLAY SETTINGS */
extern uint8_t HCYCLE[];
extern uint8_t HOFFSET[];
extern uint8_t HSYNC0[];
extern uint8_t HSYNC1[];
extern uint8_t VCYCLE[];
extern uint8_t V0FFSET[];
extern uint8_t VSYNC0[];
extern uint8_t VSYNC1[];
extern uint8_t SWIZZLE[];
extern uint8_t PCLK_POL[];
extern uint8_t CSPREAD[];
extern uint8_t HSIZE[];
extern uint8_t VSIZE[];
extern uint8_t DITHER[];
extern uint8_t GPIO_DIR[];
extern uint8_t GPIO[];
extern uint8_t PCLK[];
extern uint8_t EXTENDED_MODE[];
extern uint8_t PWM_DUTY[];

/* DRAWING */
extern uint8_t DISPLAY[];
extern uint8_t DLSWAP[];
extern uint8_t END[];

/* DRAWING MODES */
extern uint8_t BITMAPS[];
extern uint8_t POINTS[];
extern uint8_t LINES[];
extern uint8_t LINE_STRIP[];
extern uint8_t RECTS[];

/* COLORS */
extern uint8_t RTTRED[];
extern uint8_t WHITE[];
extern uint8_t BLACK[];
extern uint8_t GREEN[];
extern uint8_t RED[];
extern uint8_t ORANGE[];
extern uint8_t DARKRED[];
extern uint8_t GREY[];

// get data (for reading)
extern uint8_t getData1[];
extern uint8_t getData2[];
extern uint8_t getData3[];
extern uint8_t getData4[];

#endif /* FT812Q_FT812Q_CONSTANTS_H_ */
