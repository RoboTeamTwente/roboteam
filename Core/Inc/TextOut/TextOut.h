/*
 * TextOut.h
 *
 *  Created on: 14 sep. 2016
 *      Author: Hans-van-der-Heide
 */

#ifndef TEXTOUT_H_
#define TEXTOUT_H_

#define BYTE_TO_BINARY_PATTERN "%c%c%c%c%c%c%c%c"
#define BYTE_TO_BINARY(byte)  \
  (byte & 0x80 ? '1' : '0'), \
  (byte & 0x40 ? '1' : '0'), \
  (byte & 0x20 ? '1' : '0'), \
  (byte & 0x10 ? '1' : '0'), \
  (byte & 0x08 ? '1' : '0'), \
  (byte & 0x04 ? '1' : '0'), \
  (byte & 0x02 ? '1' : '0'), \
  (byte & 0x01 ? '1' : '0')

#include "usbd_cdc_if.h"
void LOG(char *message);
void TextOut(char *str);
void HexOut(uint8_t*, uint8_t);


#endif /* TEXTOUT_H_ */
