/*
 * FT812Q_Drawing.c
 *
 *  Created on: May 16, 2019
 *      Author: selina
 */

#include <stdlib.h>
#include <math.h>

#include "FT812Q_Drawing.h"
#include "packet_buffers.h"
#include "REM_RobotFeedback.h"


/* DATA */
uint32_t nxt;

/* DRAW ON SCREEN */
void drawBasestation(bool USBstatus){

	// Initial settings
	nxt = writeDispBuf(0,		0x4, 	CLEAR_COLOR_RGB(40, 0, 0));
	nxt = writeDispBuf(nxt, 	0x4,	CLEAR(1, 1, 1));
	nxt = writeDispBuf(nxt, 	0x4,	VERTEX_FORMAT(0));

	/* DISPLAY LIST */
	nxt = drawMenuButton(nxt, 429, 0, RTTRED);
	nxt = drawString(nxt, 97, 4, 23, "RoboTeam Twente Status Board", WHITE);
	uint16_t beginRect[] = {1, 31}; uint16_t endRect[] = {XRES, YRES};
	nxt = drawRect(nxt, beginRect, endRect, RTTRED, 2, 0); // red background rectangle

	// Draw rectangles + ID's + RX + TX
	uint8_t spacingY = 60;
	uint8_t spacingX = 120;
	uint8_t i = 0;
	char id[1];
	for (uint8_t q = 0; q < 4; q++){
		for (uint8_t p = 0; p < 4; p++){

			// set rectangle per robot id
			uint16_t begin[] = {1 + spacingX*p + 2, 31 + spacingY*q + 2};
			uint16_t end[] = {1 + spacingX*(p + 1) - 1, 31 + spacingY*(q + 1) - 2};
			if (end[0] >= 480){end[0] = 479;} // this is not a hack
			memcpy(robots[i].beginPoint, begin, 4);
			memcpy(robots[i].endPoint, end, 4);
			itoa(i, id, 10);

			nxt = drawRobotStatus(nxt, i);

			// Set RX
			// Doesn't work yet so draws a grey rectangle
			uint16_t RXBegin[] = {begin[0] + 5, begin[1] + 30};
			uint16_t RXEnd[] = {begin[0] + 54, end[1] - 10};
//			if (robots[i].robotStatus == false){
				nxt = drawRect(nxt, RXBegin, RXEnd, GREY, 2, 0);
				nxt = drawString(nxt, begin[0] + 24, begin[1] + 31, 20, "RX", WHITE);
//			} else if ((robots[i].RX_Packets < 55) || (robots[i].RX_Packets > 65)){ // not receiving enough packages
//				char RX_char[5]; sprintf(RX_char, "%i", robots[i].RX_Packets);
//				nxt = drawRect(nxt, RXBegin, RXEnd, ORANGE, 2, 0);
//				nxt = drawString(nxt, begin[0] + 24, begin[1] + 31, 20, RX_char, WHITE);
//			} else {
//				nxt = drawRect(nxt, RXBegin, RXEnd, GREEN, 2, 0);
//				nxt = drawString(nxt, begin[0] + 24, begin[1] + 31, 20, "RX", WHITE);
//			}

			// Set TX
			uint16_t TXBegin[] = {RXEnd[0] + 6, RXBegin[1]};
			uint16_t TXEnd[] = {end[0] - 6, end[1] - 10};
			if (robots[i].robotStatus == false){
				nxt = drawRect(nxt, TXBegin, TXEnd, GREY, 2, 0);
				nxt = drawString(nxt, begin[0] + 80, begin[1] + 31, 20, "TX", WHITE);
			} else if ((robots[i].TX_Packets < 55) || (robots[i].TX_Packets > 65)){ // not sending enough packages
				char TX_char[5]; sprintf(TX_char, "%i", robots[i].TX_Packets);
				nxt = drawRect(nxt, TXBegin, TXEnd, ORANGE, 2, 0);
				nxt = drawString(nxt, begin[0] + 80, begin[1] + 31, 20, TX_char, GREY);
			} else {
				nxt = drawRect(nxt, TXBegin, TXEnd, GREEN, 2, 0);
				nxt = drawString(nxt, begin[0] + 80, begin[1] + 31, 20, "TX", WHITE);
			}

			i++;
		}
	}

	// USB connection
	nxt = drawUSBicon(nxt, 5, 13, USBstatus);

	/* DRAW */
	nxt = writeDispBuf(nxt,		0x4, 	DISPLAY);
	writeDisplay(RAM_DL,		nxt, 	DispBuf);
	writeDisplay(REG_DLSWAP, 	0x1, 	DLSWAP); // Display list swap
}

uint32_t drawRobotInfo(uint8_t id, bool USBstatus){
	return 0;
	REM_RobotFeedbackPayload *rfp = NULL;//&buffer_RobotFeedback[id].packet;

	// /* CALCULATE DATA */
	float angle				= REM_RobotFeedback_get_angle(rfp);
	char angle_char[5];
	sprintf(angle_char, "%.2f", angle);

	float theta				= REM_RobotFeedback_get_theta(rfp);
	float rho 				= REM_RobotFeedback_get_rho(rfp);

	float x_vel 				= rho * cos(theta);
	float y_vel 				= rho * sin(theta);

	bool kickStatus 			= false;
	bool chipStatus 			= false;
	bool dribblerStatus			= false;

	/* INITIAL SETTINGS */
	nxt = writeDispBuf	(0,		0x4, 	CLEAR_COLOR_RGB(40, 0, 0));
	nxt = writeDispBuf	(nxt,  	0x4, 	CLEAR(1, 1, 1));
	nxt = writeDispBuf	(nxt,	0x4,	VERTEX_FORMAT(0));

	/* DISPLAY LIST */
	nxt = drawString	(nxt, 	40, 	0, 		29, 		"Robot:", 		WHITE);
	char id_value[1]; itoa(id, id_value, 10);
	nxt = drawString	(nxt, 	120, 	1, 		29, 		id_value, 		WHITE);
	nxt = drawReturn	(nxt, 	0, 		0, 		RTTRED);
	nxt = drawUSBicon	(nxt, 	430, 	13, 	USBstatus); // USB connection

	nxt = drawString	(nxt, 	15, 	50, 	18, 		"X_VEL", 		WHITE);
	nxt = drawString	(nxt, 	15, 	80, 	18, 		"Y_VEL", 		WHITE);
	nxt = drawString	(nxt, 	15, 	110, 	18, 		"ANGLE", 		WHITE);

	nxt = drawKicker	(nxt, 	15, 	200, 	kickStatus); // draw kicker status
	nxt = drawChipper	(nxt, 	70,		200,	chipStatus); // draw chipper status
	nxt = drawDribbler	(nxt, 	135, 	200, 	dribblerStatus); // draw dribbler status

	nxt = drawStatusBar	(nxt, 	120, 	50, 	MAX_X_VEL, 	x_vel); // status bar for x vel
	nxt = drawStatusBar	(nxt, 	120, 	80, 	MAX_Y_VEL, 	y_vel); // status bar for y vel
	nxt = drawStatusBar	(nxt, 	120,	110,	MAX_ANGLE, 	angle);

	nxt = drawPolarCoordinates(nxt, 300, 200, x_vel, y_vel);

	/* DRAW */
	nxt = writeDispBuf	(nxt, 	0x4, 	DISPLAY);
	writeDisplay(RAM_DL, 		nxt, 	DispBuf);
	writeDisplay(REG_DLSWAP, 	0x1, 	DLSWAP); // Display list swap
	return nxt;
}

uint32_t drawBrightnessSlider(TouchState *touchState){
	if(touchState->state != TOUCH_STATE_PRESSED)
		return 0;

	/* INITIAL SETTINGS */
	nxt = writeDispBuf	(0,		0x4, 	CLEAR_COLOR_RGB(40, 0, 0));
	nxt = writeDispBuf	(nxt,  	0x4, 	CLEAR(1, 1, 1));
	nxt = writeDispBuf	(nxt,	0x4,	VERTEX_FORMAT(0));

	// Draw large bar
	uint16_t y = YRES/2-5;
	uint16_t begin[] 	= {40, y};
	uint16_t end[] 		= {XRES-40, y};
	nxt = drawRect(nxt, begin, end, WHITE, 2, 1);

	// Draw slider square
	uint16_t px = touchState->x;
	if(px < 40) px = 40;
	if(XRES-40 < px) px = XRES-40;
	begin[0] = px-5; begin[1] = y-10;
	end[0]   = px+5; end[1] = y+10;
	nxt = drawRect(nxt, begin, end, WHITE, 2, 1);

	/* DRAW */
	nxt = writeDispBuf	(nxt, 	0x4, 	DISPLAY);
	writeDisplay(RAM_DL, 		nxt, 	DispBuf);
	writeDisplay(REG_DLSWAP, 	0x1, 	DLSWAP); // Display list swap

	uint8_t pwm = 128 * (((float)px-40) / ((float)XRES-80));
	writeDisplay(REG_PWM_DUTY, 0x1,	&pwm);

	return nxt;
}

/* SUPPORTING FUNCTIONS */
uint32_t drawString(uint32_t addr, uint16_t x, uint16_t y, uint8_t handle, char* string, uint8_t color[]){
	addr = writeDispBuf(addr,  	0x4, 	COLOR_RGB(color[0], color[1], color[2]));
	addr = writeDispBuf(addr, 	0x4, 	BITMAPS);

	int i = 0;
	uint8_t* romfont;
	uint32_t width;
	uint8_t* spacing;
	while(string[i] != '\0'){
		addr = writeDispBuf(addr, 0x4, LETTER_DATA(x, y, handle, string[i]));
		romfont = readDisplay(ROM_FONTROOT, 0x4, getData4);
		width = *(uint32_t*)romfont + (148 * (handle - 16));
		spacing = readDisplay(((uint32_t)width + string[i]), 0x1, getData1);
		x+= *spacing;
		i++;
	}
	return addr;	// return the address at which the next instruction should be
}

uint32_t drawRect(uint32_t addr, uint16_t begin[], uint16_t end[], uint8_t color[], uint8_t width, uint8_t option){
	if (option == 0){
		addr = writeDispBuf(addr,  	0x4, 	COLOR_RGB(color[0], color[1], color[2]));
		addr = writeDispBuf(addr,	0x4, 	RECTS);
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(begin[0], begin[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(end[0], end[1]));
	} else {
		addr = writeDispBuf(addr,	0x4, 	LINES);
		addr = writeDispBuf(addr,  	0x4, 	COLOR_RGB(color[0], color[1], color[2]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(begin[0], begin[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(end[0], begin[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(end[0], begin[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(end[0], end[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(end[0], end[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(begin[0], end[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(begin[0], end[1]));
		addr = writeDispBuf(addr,	0x4, 	POINT_DATA(begin[0], begin[1]));
		addr = writeDispBuf(addr,	0x4,	LINE_WIDTH(width));
	}
	return addr;
}

uint32_t drawLine(uint32_t addr, uint16_t xb, uint16_t yb, uint16_t xe, uint16_t ye, uint8_t color[], uint16_t width){
	addr = writeDispBuf(addr,  	0x4, 	COLOR_RGB(color[0], color[1], color[2]));
	addr = writeDispBuf(addr,	0x4,	LINE_WIDTH(width));
	addr = writeDispBuf(addr, 	0x4, 	LINES);
	addr = writeDispBuf(addr,	0x4, 	POINT_DATA(xb, yb));
	addr = writeDispBuf(addr,	0x4,	POINT_DATA(xe, ye));
	return addr;
}

uint32_t drawDot(uint32_t addr, uint16_t x, uint16_t y, uint8_t size, uint8_t color[]){
	addr = writeDispBuf(addr, 	0x4, 	COLOR_RGB(color[0], color[1], color[2]));
	addr = writeDispBuf(addr, 	0x4, 	POINT_SIZE(size));
	addr = writeDispBuf(addr, 	0x4, 	POINTS);
	addr = writeDispBuf(addr, 	0x4, 	POINT_DATA(x, y));
	return addr;
}

uint32_t drawUSBicon(uint32_t addr, uint16_t x, uint16_t y, bool USBstatus){
	uint8_t color[3];
	if (USBstatus == 1){
		memcpy(color, GREEN, 3);
	} else
		memcpy(color, RED, 3);

	addr = drawDot(addr, 	x, 		y, 		80, color); // dot left
	addr = drawDot(addr, 	x + 40, y, 		60, color); // dot right
	addr = drawLine(addr, 	x, 		y, 		x + 40,		y, color, 		2); // line to the right
	addr = drawLine(addr, 	x + 10, y, 		x + 15, 	y - 8, color, 	2); // line above
	addr = drawLine(addr, 	x + 15, y - 8, 	x + 25, 	y - 8, color, 	2); // line above
	addr = drawDot(addr, 	x + 25, y - 8, 	60, color); // dot above
	addr = drawLine(addr, 	x + 15, y, 		x + 20, 	y + 8, color, 	2); // line below
	addr = drawLine(addr, 	x + 20, y + 8, 	x + 30, 	y + 8, color, 	2); // line below
	addr = drawDot(addr, 	x + 30, y + 8, 	60, color); // dot below

	return addr;
}

uint32_t drawRobotStatus(uint32_t addr, uint8_t id){
	char id_char[1]; itoa(id, id_char, 10);

	/* DISPLY LIST */
	addr = drawRect(addr, robots[id].beginPoint, robots[id].endPoint, DARKRED, 2, 0); // background rectangle
	addr = drawString(addr, robots[id].beginPoint[0] + 5, robots[id].beginPoint[1] + 2, 23, id_char, WHITE);
	if (robots[id].robotStatus == true){
		addr = drawString(addr, robots[id].beginPoint[0] + 40, robots[id].beginPoint[1] + 3, 22, "Online", GREEN);
	} else {
		addr = drawString(addr, robots[id].beginPoint[0] + 40, robots[id].beginPoint[1] + 3, 22, "Offline", WHITE);
	}
	return addr;
}

uint32_t drawReturn(uint32_t addr, uint16_t x, uint16_t y, uint8_t color[]){
	uint16_t begin[] = {x,y};
	uint16_t end[] = {x + 30,y + 30};
	addr = drawRect(addr, 	begin, 		end, 		color, 		2, 			1);
	addr = drawLine(addr, 	x + 5, 		y + 15, 	x + 25, 	y + 15, 	color, 		2);
	addr = drawLine(addr, 	x + 5, 		y + 15, 	x + 15, 	y + 10, 	color, 		2);
	addr = drawLine(addr, 	x + 5, 		y + 15, 	x + 15, 	y + 20, 	color, 		2);
	return addr;
}

uint32_t drawKicker(uint32_t addr, uint16_t x, uint16_t y, bool kickStatus){
	uint8_t color[3];
	if (kickStatus == 1){
		memcpy(color, GREEN, 3);
	} else
		memcpy(color, RED, 3);

	uint8_t barWidth 	= 10;
	uint8_t barLength 	= 31;
	uint16_t begin[] 	= {x, y};
	uint16_t end[] 		= {x + barLength, y + barWidth};

	addr = drawRect(addr, begin, end, color, 2, 0); // draw kicker bar
	begin[0] 	= begin[0] 	+ barWidth;
	begin[1] 	= end[1];
	end[0] 		= end[0] 	- barWidth;
	end[1] 		= begin[1] 	+ barLength - 9;
	addr = drawRect(addr, begin, end, color, 2, 0); // draw bar that does the kicking
	return addr;
}

uint32_t drawChipper(uint32_t addr, uint16_t x, uint16_t y, bool chipStatus){
	uint8_t color[3];
	if (chipStatus == 1){
		memcpy(color, GREEN, 3);
	} else
		memcpy(color, RED, 3);

	uint8_t barWidth 	= 10;
	uint8_t barLength 	= 40;
	uint8_t armWidth 	= 3;
	uint8_t armLength 	= 22;

	// Draw arms
	uint16_t beginArm[] = {x, y};
	uint16_t endArm[] 	= {x + armWidth, y + armLength};
	addr = drawRect(addr, beginArm, endArm, WHITE, 2, 0);
	beginArm[0] 		= x 		+ barLength - armWidth;
	endArm[0] 			= endArm[0] + barLength - armWidth;
	addr = drawRect(addr, beginArm, endArm, WHITE, 2, 0);

	// Draw plate
	uint16_t begin[] 	= {x, y + 22};
	uint16_t end[] 		= {x + barLength, begin[1] + barWidth};
	addr = drawRect(addr, begin, end, color, 2, 0);
	return addr;
}

uint32_t drawDribbler(uint32_t addr, uint16_t x, uint16_t y, bool dribblerStatus){
	uint8_t color[3];
	if (dribblerStatus == 1){
		memcpy(color, GREEN, 3);
	} else
		memcpy(color, RED, 3);

	uint8_t barLength 	= 40;
	uint8_t barWidth 	= 10;
	uint16_t begin[] 	= {x, y};
	uint16_t end[] 		= {x + barLength, y + barWidth};

	addr = drawRect	(addr, begin, end, color, 2, 0);
	addr = drawDot	(addr, (end[0] - x)/2 + x, end[1] + 12, 170, ORANGE);
	return addr;
}

uint32_t drawStatusBar(uint32_t addr, uint16_t x, uint16_t y, uint16_t max, float value){
	uint8_t barLength 	= 200;
	uint8_t barWidth 	= 10;
	uint16_t begin[] 	= {x, y + 1};
	uint16_t end[] 		= {x + barLength, y + barWidth + 1};
	addr = drawRect(addr, begin, end, WHITE, 2, 1);

	if (max == MAX_X_VEL || max == MAX_Y_VEL){
		char max_value[1]; itoa((int)max, max_value, 10); // convert max to string to draw
		addr = drawString(addr, 	x - 20, 		y, 	18, 	"-8", 		RTTRED);
		addr = drawString(addr, 	end[0] + 8, 	y, 	18, 	max_value, 	RTTRED);
		if (value > 0){
			begin[0] = x + barLength/2;
			end[0] = x + barLength/2 + (value/(float)max)*(barLength/2);
		} else {
			end[0] = x + barLength/2;
			begin[0] = x + 100 - ((value/(float)max)*(barLength/2)*-1);
		}
		addr = drawRect(addr, begin, end, WHITE, 2, 0);
	} else { // draw angle
		addr = drawString(addr, 	x - 43, 		y, 	18, 	"-3.14", 	RTTRED);
		addr = drawString(addr, 	end[0] + 8, 	y, 	18, 	"3.14", 	RTTRED);
		if (value > 0){
			begin[0] = x + barLength/2;
			end[0] = x + barLength/2 + (value/(float)max)*(barLength/2);
		} else {
			end[0] = x + barLength/2;
			begin[0] = x + 100 - ((value/(float)max)*(barLength/2)*-1);
		}
		addr = drawRect(addr, begin, end, WHITE, 2, 0);
	}

	char value_value[4]; sprintf(value_value, "%.1f", value);
	addr = drawString(addr, x + 250, y, 18, value_value, WHITE);
	return addr;
}

uint32_t drawMenuButton(uint32_t addr, uint16_t x, uint16_t y, uint8_t color[]){
	uint16_t begin[] = {x, y};
	uint16_t end[] = {x + 50,y + 31};
	addr = drawRect(addr, begin, end, color, 2, 1);
	addr = drawString(addr, x + 2, y + 4, 23, "Menu", color);
	return addr;
}

uint32_t drawPolarCoordinates(uint32_t addr, uint16_t x, uint16_t y, float x_vel, float y_vel){
	uint8_t axisLength = 100;
	x_vel = x_vel * 6;
	y_vel = y_vel * 6;
	addr = drawLine(addr, x, y, x + axisLength, y, RTTRED, 2);
	addr = drawLine(addr, x + axisLength/2, y - axisLength/2, x + axisLength/2, y + axisLength/2, RTTRED, 2);
	addr = drawLine(addr, x + axisLength/2, y, (int16_t)(x + axisLength/2 + x_vel), (int16_t)(y + y_vel), WHITE, 2);
	return addr;
}
