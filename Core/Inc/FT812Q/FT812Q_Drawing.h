/*
 * FT812Q_Drawing.h
 *
 *  Created on: May 16, 2019
 *      Author: selina
 */

#ifndef FT812Q_FT812Q_DRAWING_H_
#define FT812Q_FT812Q_DRAWING_H_

#include "FT812Q.h"
#include "FT812Q_Constants.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

/* FUNCTIONS */
void drawBasestation(bool USBstatus);
uint32_t drawRobotInfo(uint8_t id, bool USBstatus);
uint32_t drawString(uint32_t addr, uint16_t x, uint16_t y, uint8_t handle, char* string, uint8_t color[]);
uint32_t drawRect(uint32_t addr, uint16_t begin[], uint16_t end[], uint8_t color[], uint8_t width, uint8_t option); // 0 = filled, 1 = lines
uint32_t drawLine(uint32_t addr, uint16_t xb, uint16_t yb, uint16_t xe, uint16_t ye, uint8_t color[], uint16_t width);
uint32_t drawDot(uint32_t addr, uint16_t x, uint16_t y, uint8_t size, uint8_t color[]);
uint32_t drawUSBicon(uint32_t addr, uint16_t x, uint16_t y, bool USBstatus);
uint32_t drawRobotStatus(uint32_t addr, uint8_t id);
uint32_t drawReturn(uint32_t addr, uint16_t x, uint16_t y, uint8_t color[]);
uint32_t drawKicker(uint32_t addr, uint16_t x, uint16_t y, bool kickStatus);
uint32_t drawChipper(uint32_t addr, uint16_t x, uint16_t y, bool chipStatus);
uint32_t drawDribbler(uint32_t addr, uint16_t x, uint16_t y, bool dribblerStatus);
uint32_t drawStatusBar(uint32_t addr, uint16_t x, uint16_t y, uint16_t max, float value);
uint32_t drawMenuButton(uint32_t addr, uint16_t x, uint16_t y, uint8_t color[]);
uint32_t drawPolarCoordinates(uint32_t addr, uint16_t x, uint16_t y, float x_vel, float y_vel);

/* ROBOT DATA */
typedef struct drawingData {
	uint16_t beginPoint[2];
	uint16_t endPoint[2];
	bool robotStatus;
	int TX_Packets;
	int RX_Packets;
}drawingData;

drawingData robots[16];

// this is not the right place for this
#define MAX_DRIBBLE_VEL 	7
#define MAX_ANGLE			3.14
#define MAX_X_VEL			8
#define MAX_Y_VEL			8
#define CONVERT_RHO			0.004f
#define CONVERT_THETA		0.00307f
#define CONVERT_ANGLE		0.00613f


#endif /* FT812Q_FT812Q_DRAWING_H_ */
