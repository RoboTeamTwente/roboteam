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

#ifndef __XBUSMESSAGE_H
#define __XBUSMESSAGE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/*! \brief Xbus message IDs. */
enum XsMessageId
{
	//8.1 WakeUp and State messages,//Section 5.3.1)
	//XMID_Message MID Direction
	XMID_WakeUp 					= 62,//0x3E) To host
	XMID_WakeUpAck 					= 63,//0x3F) To MT
	XMID_GoToConfig 				= 48,//0x30) To MT
	XMID_GoToConfigAck				= 49,//0x31) To host
	XMID_GoToMeasurement 			= 16,//0x10) To MT
	XMID_GoToMeasurementAck 		= 17,//0x11) To host
	XMID_Reset 						= 64,//0x40) To MT
	XMID_ResetAck 					= 65,//0x41) To host
	//8.2 Informational messages,//Section 5.3.2)
	XMID_ReqDID 					= 0,//0x00) To MT Host request device ID of the device
	XMID_DeviceID 					= 1,//0x01) To host Device acknowledges request by sending its ID
	XMID_ReqProductCode 			= 28,//0x1c) To MT Host request product code of the device
	XMID_ProductCode 				= 29,//0x1d) To host Device acknowledges request by sending its product code
	XMID_ReqFWRev 					= 18,//0x12) To MT Host requests firmware revision of device
	XMID_FirmwareRev 				= 19,//0x13) To host Device acknowledges request by sending its firmware revision
	XMID_Error 						= 66,//0x42) To host Error message
	//8.3 Device-specific messages,//Section 5.3.3)
	XMID_RestoreFactoryDef 			= 14,//0xE) To MT Restores all settings in MT to factory defaults
	XMID_ReqBaudrate 				= 24,//0x18) To MT Requests current baud rate of the serial communication
	XMID_ReqBaudrateAck 			= 25,//0x19) To host Device returns baud rate of serial communication
	XMID_SetBaudrate 				= 24,//0x18) To MT Host sets baud rate of serial communication
	XMID_SetBaudrateAck 			= 25,//0x19) To host Device acknowledges SetBaudrate message
	XMID_RunSelftest 				= 36,//0x24) To MT Runs the built-in self test
	XMID_SelftestAck 				= 37,//0x25) To host Returns the self test results
	XMID_ReqGnssPlatform 			= 118,//0x76) To MT Requests the current GNSS platform setting
	XMID_ReqGnssPlatformAck 		= 119,//0x76) To host Returns the current GNSS platform setting
	XMID_SetGnssPlatform 			= 118,//0x76) To MT Sets the GNSS platform setting
	XMID_SetGnssPlatformAck 		= 119,//0x76) To host Acknowledges setting of GNSS platform setting
	XMID_ReqErrorMode 				= 218,//0xDA) To MT Request error mode
	XMID_ReqErrorModeAck 			= 219,//0xDB) To host Device returns error mode
	XMID_SetErrorMode 				= 218,//0xDA) To MT Host sets error mode
	XMID_SetErrorModeAck 			= 219,//0xDB) To host Device acknowledges SetErrorMode message
	XMID_ReqTransmitDelay 			= 220,//0xDC) To MT Request the transmit delay in RS485 MT’s
	XMID_ReqTransmitDelayAck 		= 221,//0xDD) To host Device returns the transmit delay in RS485 MT’s
	XMID_SetTransmitDelay 			= 220,//0xDC) To MT Host sets transmit delay in RS485 MT’s
	XMID_SetTransmitDelayAck 		= 221,//0xDD) To host Device acknowledges SetTransmitDelay message
	XMID_ReqOptionFlags 			= 72,//0x48) To MT Requests state of OptionFlags
	XMID_ReqOptionFlagsAck 			= 73,//0x49) To host Device returns OptionFlags
	XMID_SetOptionFlags 			= 72,//0x48) To MT Sets state of OptionFlags
	XMID_SetOptionFlagsAck 			= 73,//0x49) To host Device acknowledges SetOptionFlags message
	XMID_ReqLocationID 				= 132,//0x84) To MT Request location ID
	XMID_ReqLocationIDAck 			= 133,//0x85) To host Device returns location ID
	XMID_SetLocationID 				= 132,//0x84) To MT Host sets location ID
	XMID_SetLocationIDAck 			= 133,//0x85) To host Device acknowledges SetLocationID message
	//8.4 Synchronization messages,//Section 5.3.4)
	XMID_ReqSyncSettings 			= 44,//0x2C) To MT Request the synchronization settings of the device
	XMID_ReqSyncSettingsAck 		= 45,//0x2D) To host Device returns synchronization settings
	XMID_SetSyncSettings 			= 44,//0x2C) To MT Set the synchronization settings of the device
	XMID_SetSyncSettingsAck 		= 45,//0x2D) To host Device acknowledges SetSyncSettings
	//8.5 Configuration messages,//Section 5.3.5)
	XMID_ReqConfiguration 			= 12,//0x0C) To MT Request the configuration of device. For logging/quick setup purposes
	XMID_Configuration 				= 13,//0x0D) To host Contains the configuration of device
	XMID_ReqPeriod 					= 4,//0x04) To MT Request current sampling period
	XMID_ReqPeriodAck 				= 5,//0x05) To host Device returns sampling period
	XMID_SetPeriod 					= 4,//0x04) To MT Host sets sampling period,//10-500Hz)
	XMID_SetPeriodAck 				= 5,//0x05) To host Device acknowledges SetPeriod message
	XMID_ReqExtOutputMode 			= 134,//0x86) To MT Requests the current extended output mode
	XMID_ExtOutputMode 				= 135,//0x87) To host Device returns the current extended output mode
	XMID_SetExtOutputMode 			= 134,//0x86) To MT Sets the extended output mode
	XMID_SetExtOutputModeAck 		= 135,//0x87) To host Device acknowledges SetExtOutputMode
	XMID_ReqOutputConfiguration 	= 192,//0xC0) To MT Request the current output configuration
	XMID_ReqOutputConfigurationAck 	= 193,//0xC1) To Host Device returns the output configuration
	XMID_SetOutputConfiguration 	= 192,//0xC0) To MT Sets the output configuration
	XMID_SetOutputConfigurationAck 	= 193,//0xC1) To Host Device acknowledges SetOutputconfiguration message
	XMID_ReqStringOutputTypes 		= 142,//0x8E) To MT Request the configuration of the NMEA data output
	XMID_ReqStringOutputTypesAck 	= 143,//0x8F) To host Device returns the NMEA output configuration
	XMID_SetStringOutputTypes 		= 142,//0x8E) To MT Configures the NMEA data output
	XMID_SetStringOutputTypesAck 	= 143,//0x8F) To host Device acknowledges SetStringOutputTypes message
	XMID_ReqAlignmentRotation 		= 236,//0xEC) To MT Requests the sensor alignment or local alignment
	XMID_ReqRotationQuaternionAck 	= 237,//0xED) To host Device acknowledges ReqRotationQuaternion
	XMID_SetAlignmentRotation 		= 236,//0xEC) To MT Sets the sensor alignment or local alignment
	XMID_SetRotationQuaternionAck 	= 237,//0xED) To host Device acknowledges SetRotationQuaternion
	//8.6 Data-related messages,
	XMID_ReqData 					= 52,//0x34) To MT Host requests device to send MTData2 message
	XMID_MTData2 					= 54,//0x36) To host Message with one or more output data packets
	//8.7 Filter messages,//Section 5.3.7)
	XMID_ResetOrientation 			= 164,//0xA4) To MT Resets the orientation
	XMID_ResetOrientationAck 		= 165,//0xA5) To host Device acknowledges ResetOrientation message
	XMID_ReqUTCTime 				= 96,//0x60) To MT Request UTC Time
	XMID_SetUTCTime					= 96,//0x60) To MT Sets time in UTC format
	XMID_AdjustUTCTime 				= 168,//0xA8) To MT Sets correction ticks to UTC time
	XMID_UTCTime 					= 97,//0x61) To host Device return UTC Time
	XMID_ReqAvailableFilterProfiles = 98,//0x62) To MT Request available filter profiles
	XMID_AvailableFilterProfiles 	= 99,//0x63) To host Device return available filter profiles
	XMID_ReqFilterProfile 			= 100,//0x64) To MT Request current used filter profile
	XMID_ReqFilterProfileAck 		= 101,//0x65) To host Device return current filter profile
	XMID_SetFilterProfile 			= 100,//0x64) To MT Host set current filter profile
	XMID_SetFilterProfileAck 		= 101,//0x65) To host Device acknowledges SetFilterProfile
	XMID_ReqLatLonAlt 				= 110,//0x6E) To MT Requests the latitude, longitude and altitude that is stored in the device
	XMID_ReqLatLonAltAck 			= 111,//0x6F) To host Returns the latitude, longitude and altitude that is stored in the device
	XMID_SetLatLonAlt 				= 110,//0x6E) To host Sets latitude, longitude and altitude in the device
	XMID_SetLatLonAltAck 			= 111,//0x6F) To MT Device acknowledges SetLatLonAlt
	XMID_SetNoRotation 				= 34,//0x22) To MT Initiates ‘no rotation’ update procedure
	XMID_SetNoRotationAck 			= 35,//0x23) To host Device acknowledges SetNoRotation XMID_message
	XMID_IccCommand 				= 116,//0x74) To MT
	XMID_IccCommandAck 				= 117//0x75) To host
};

/*! \brief Xbus data message type IDs. */
enum XsDataIdentifier
{
	XDI_Temperature		= 0x0810,
	XDI_UtcTime			= 0x1010,
	XDI_PacketCounter	= 0x1020,
	XDI_SampleTimeFine	= 0x1060,
	XDI_SampleTimeCoarse= 0x1070,
	XDI_Quaternion		= 0x2010,// not on MTi 1
	XDI_RotationMatrix	= 0x2020,// not on MTi 1
	XDI_EulerAngles		= 0x2030,// not on MTi 1
	XDI_DeltaV			= 0x4010,
	XDI_Acceleration	= 0x4020,
	XDI_FreeAcceleration= 0x4030,
	XDI_AccelerationHR	= 0x4040,
	XDI_RateOfTurn		= 0x8020,
	XDI_DeltaQ			= 0x8030,
	XDI_RateOfTurnHR	= 0x8040,
	XDI_MagneticField	= 0xC020,
	XDI_StatusByte		= 0xE010,
	XDI_StatusWord		= 0xE020
};

enum XsFormatPrecision{
	XDI_Float32	= 0x0,
	XDI_Fp1220	= 0x1,
	XDI_Fp1632	= 0x2,
	XDI_Float64	= 0x3
};

enum XsFormatCoordinateSystem{
	XDI_ENU		= 0x0,//use this one
	XDI_NED		= 0x4,
	XDI_NWU		= 0x8
};

enum XsOptionFlags{
	XOF_DisableAutoStore				= 0x00000001U,
	XOF_DisableAutoMeasurement 			= 0x00000002U,
	XOF_EnableAhs						= 0x00000010U,
	XOF_EnableInRunCompassCalibration	= 0x00000080U
};

enum XsIcc{
	XsIcc_Start  	= 0,	//Representative Motion | Measurement State
	XsIcc_Stop		= 1, 	//Representative Motion | Measurement State
	XsIcc_Store		= 2,	//ICC parameters Config | State
	XsIcc_Get		= 3,	//Representative Motion | State Measurement State
};

enum XsFilterProfile{
	XFP_General 		= 50,//Suitable for most applications.
	XFP_High_mag_dep 	= 51,//Heading corrections strongly rely on the magnetic field measured and should be used when magnetic field is homogeneous.
	XFP_Dynamic			= 52,//Assumes that the motion is highly dynamic.
	XFP_North_referenced= 53,//Assumes a good Magnetic Field Mapping (MFM) and a homogeneous magnetic field. Given stable initialization procedures and observability of the gyro bias, after dynamics,
						//this filter profile will trust more on the gyro solution and the heading will slowly converge to the disturbed mag field over the course of time.
	XFP_VRU_general		= 54//Magnetometers are not used to determine heading. Consider using VRU_general in environments that have a heavily disturbed magnetic field or when the application only requires
						//unreferenced heading (see also Section 4.3.3)
};
/*!
 * \brief Low level format to use when formating Xbus messages for transmission.
 */
enum XbusLowLevelFormat
{
	/*! \brief Format for use with I2C interface. */
	XLLF_I2c,
	/*! \brief Format for use with SPI interface. */
	XLLF_Spi,
	/*! \brief Format for use with UART interface. */
	XLLF_Uart
};

/*!
 * \brief An Xbus message structure with optional payload.
 */
struct XbusMessage
{
	/*! \brief The message ID of the message. */
	enum XsMessageId mid;
	/*!
	 * \brief The length of the payload.
	 *
	 * \note The meaning of the length is message dependent. For example,
	 * for XMID_OutputConfig messages it is the number of OutputConfiguration
	 * elements in the configuration array.
	 */
	uint16_t length;
	/*! \brief Pointer to the payload data. */
	void* data;
};

/*!
 * \brief Output configuration structure.
 */
struct OutputConfiguration
{
	/*! \brief Data type of the output. */
	enum XsDataIdentifier dtype;
	/*!
	 * \brief The output frequency in Hz, or 65535 if the value should be
	 * included in every data message.
	 */
	uint16_t freq;
};

size_t XbusMessage_format(uint8_t* raw, struct XbusMessage const* message, enum XbusLowLevelFormat format);
uint16_t XbusMessage_getOutputFreq(enum XsDataIdentifier id, struct XbusMessage const* message);
bool XbusMessage_getDataItem(void* item, enum XsDataIdentifier id, struct XbusMessage const* message);
char const* XbusMessage_dataDescription(enum XsDataIdentifier id);

#ifdef __cplusplus
}
#endif // extern "C"

#endif // __XBUSMESSAGE_H
