/*
 * FT812Q_Constants.c
 *
 *  Created on: May 16, 2019
 *      Author: selina
 */

#include "FT812Q_Constants.h"

#include <stdio.h>
#include <string.h>

/* DATA */
/* DISPLAY RESOLUTION */
uint16_t XRES = 480;
uint16_t YRES = 272;

/* BUFFER */
uint8_t buffer[DISP_BUF_LENGTH] __attribute__((aligned(4)));
uint8_t* DispBuf = buffer;

/* POWER MODES */
uint8_t POWERDOWN[] 	= {0x50, 0x0, 0x0};
uint8_t SLEEP[] 		= {0x41, 0x0, 0x0};
uint8_t STANDBY[]	 	= {0x41, 0x0, 0x0};
uint8_t ACTIVE[] 		= {0x0, 0x0, 0x0};

/* DISPLAY SETTINGS */
uint8_t HCYCLE[] 		= {0x24, 0x2};
uint8_t HOFFSET[]		= {0x2B, 0x0};
uint8_t HSYNC0[] 		= {0x0, 0x0};
uint8_t HSYNC1[] 		= {0x29, 0x0};
uint8_t VCYCLE[] 		= {0x24, 0x1};
uint8_t V0FFSET[]		= {0xC, 0x0};
uint8_t VSYNC0[] 		= {0x0, 0x0};
uint8_t VSYNC1[] 		= {0xA, 0x0};
uint8_t SWIZZLE[] 		= {0x0};
uint8_t PCLK_POL[] 		= {0x1};
uint8_t CSPREAD[] 		= {0x1};
uint8_t HSIZE[] 		= {0xE0, 0x1};
uint8_t VSIZE[] 		= {0x10, 0x1};
uint8_t DITHER[] 		= {0x1};
uint8_t GPIO_DIR[] 		= {0x80};
uint8_t GPIO[] 			= {0x80};
uint8_t PCLK[] 			= {0x5};
uint8_t EXTENDED_MODE[]	= {0x0};
uint8_t PWM_DUTY[] 		= {0x80};

/* DRAWING */
uint8_t DISPLAY[] 		= {0x0, 0x0, 0x0, 0x0};
uint8_t DLSWAP[] 		= {0x2};
uint8_t END[] 			= {0x0, 0x0, 0x0, 0x21};

/* DRAWING MODES */
uint8_t BITMAPS[] 		= {0x1, 0x0, 0x0, 0x1F};
uint8_t POINTS[] 		= {0x2, 0x0, 0x0, 0x1F};
uint8_t LINES[] 		= {0x3, 0x0, 0x0, 0x1F};
uint8_t LINE_STRIP[] 	= {0x4, 0x0, 0x0, 0x1F};
uint8_t RECTS[] 		= {0x9, 0x0, 0x0, 0x1F};

/* COLORS */
uint8_t RTTRED[]		= {0xCC, 0x0, 0x0};
uint8_t WHITE[]			= {0xFF, 0xFF, 0xFF};
uint8_t BLACK[]			= {0x0, 0x0, 0x0};
uint8_t GREEN[]			= {0x0C, 0xFF, 0x0};
uint8_t RED[]			= {0xFF, 0x0, 0x0};
uint8_t ORANGE[]		= {0xFF, 0x83, 0x0};
uint8_t DARKRED[]		= {0x28, 0x0, 0x0};
uint8_t GREY[]			= {0x30, 0x31, 0x33};

// get data (for reading)
uint8_t getData1[] 	= {0x0};
uint8_t getData2[] 	= {0x0};
uint8_t getData3[] 	= {0x0};
uint8_t getData4[] 	= {0x0};
