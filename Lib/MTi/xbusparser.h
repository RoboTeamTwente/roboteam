/*!
 * \file
 * \copyright Copyright (C) Xsens Technologies B.V., 2015.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy
 * of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#ifndef __XBUSPARSER_H
#define __XBUSPARSER_H

#include <MTi/xbusmessage.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief XbusParser states. */
enum XbusParserState
{
	XBPS_Preamble,          /*!< \brief Looking for preamble. */
	XBPS_BusId,             /*!< \brief Waiting for bus ID. */
	XBPS_MessageId,         /*!< \brief Waiting for message ID. */
	XBPS_Length,            /*!< \brief Waiting for length. */
	XBPS_ExtendedLengthMsb, /*!< \brief Waiting for extended length MSB*/
	XBPS_ExtendedLengthLsb, /*!< \brief Waiting for extended length LSB*/
	XBPS_Payload,           /*!< \brief Reading payload. */
	XBPS_Checksum           /*!< \brief Waiting for checksum. */
};

/*!
 * \brief Callback function structure for use with the XbusParser.
 */
struct XbusParserCallback
{
	/*!
	 * \brief Allocate a buffer for message reception.
	 * \param bufSize The size of the buffer to allocate.
	 * \returns Pointer to buffer to use for message reception, or NULL if
	 * a buffer cannot be allocated.
	 *
	 * \note It is the resposibility of the user to deallocate the message
	 * data buffers pointed to by XbusMessage structures passed to the
	 * handleMessage() callback function.
	 */
	void* (*allocateBuffer)(size_t bufSize);

	/*!
	 * \brief Deallocate a buffer that was previously allocated by a call to
	 * allocateBuffer.
	 */
	void (*deallocateBuffer)(void const* buffer);

	/*!
	 * \brief Handle a received message.
	 *
	 * \note If the passed XbusMessage structure has a non-null data pointer
	 * then it is the responsibility of the user to free this once handling
	 * of the message is complete.
	 */
	void (*handleMessage)(struct XbusMessage const* message);
};

/*!
 * \brief Xbus Parser state structure.
 */
struct XbusParser
{
	/*! \brief Callbacks for memory management, and message handling. */
	struct XbusParserCallback callbacks;
	/*! \brief Storage for the current message being received. */
	struct XbusMessage currentMessage;
	/*! \brief The number of bytes of payload received for the current message. */
	uint16_t payloadReceived;
	/*! \brief The calculated checksum for the current message. */
	uint8_t checksum;
	/*! \brief The state of the parser. */
	enum XbusParserState state;
};

size_t XbusParser_mem(void);
struct XbusParser* XbusParser_create(struct XbusParserCallback const* callback);
void XbusParser_destroy(struct XbusParser* parser);
struct XbusParser* XbusParser_init(void* parserMem, struct XbusParserCallback const* callback);

void XbusParser_parseByte(struct XbusParser* parser, uint8_t byte);
void XbusParser_parseBuffer(struct XbusParser* parser, uint8_t const* buf, size_t bufSize);
void TheAlligator(struct XbusParser* parser);


#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSPARSER_H
