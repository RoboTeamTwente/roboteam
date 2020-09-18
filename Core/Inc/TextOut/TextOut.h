/*
 * TextOut.h
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#ifndef TEXTOUT_H_
#define TEXTOUT_H_

#include "usbd_cdc_if.h"
extern char smallStrBuffer[1024];
void TextOut(char *str);
void HexOut(uint8_t*, uint8_t);


#endif /* TEXTOUT_H_ */
