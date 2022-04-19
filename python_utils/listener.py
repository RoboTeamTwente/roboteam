import time
import math
from inspect import getmembers
import utils
import serial
import numpy as np
import re
import argparse
import sys 
import shutil
import multiprocessing

import roboteam_embedded_messages.python.REM_BaseTypes as REM_BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo


def printPacket(rc):
	maxLength = max([len(k) for k, v in getmembers(rc)])
	title = re.findall(r"_(\w+) ", str(rc))[0]
	
	lines = [("┌─ %s "%title) + ("─"*100)[:maxLength*2+2-len(title)] + "┐" ]	
	lines += [ "│ %s : %s │" % ( k.rjust(maxLength) , str(v).ljust(maxLength) ) for k, v in getmembers(rc) ]
	lines += [ "└" + ("─"*(maxLength*2+5)) + "┘"]
	print("\n".join(lines))

serial_connection = None

robotCommand = REM_RobotCommand()
robotFeedback = REM_RobotFeedback()
robotStateInfo = REM_RobotStateInfo()

feedbackTimestamp = 0
stateInfoTimestamp = 0

# stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF525750877267181714-if02"
# stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF544852707267223637-if02"

while True:
	# Open serial_connection with the serial_connection
	if serial_connection is None or not serial_connection.isOpen():
		serial_connection = utils.openContinuous(timeout=0.1)
		# serial_connection = utils.openContinuous(timeout=0.001)

	try:
		# Continuously read and print messages from the serial_connection
		while True:
			msg = serial_connection.readline()
			if len(msg) == 0:
				continue
			print(msg.decode())
			continue

			### Read any packets coming from the serial_connection
			# Read packet type
			packet_type = serial_connection.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			# Parse packet based on packet type
			if packetType == REM_BaseTypes.PACKET_TYPE_REM_ROBOT_FEEDBACK:
				feedbackTimestamp = time.time()
				packet = packet_type + serial_connection.read(REM_BaseTypes.PACKET_SIZE_REM_ROBOT_FEEDBACK - 1)
				# robotFeedback.decode(packet)
				print("[PACKET_TYPE_REM_ROBOT_FEEDBACK]")

			elif packetType == REM_BaseTypes.PACKET_TYPE_REM_ROBOT_STATE_INFO:
				stateInfoTimestamp = time.time()
				packet = packet_type + serial_connection.read(REM_BaseTypes.PACKET_SIZE_REM_ROBOT_STATE_INFO - 1)
				# robotStateInfo.decode(packet)
				print("[ROBOT_STATE_INFO]")

			elif packetType == REM_BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG:
				logmessage = serial_connection.readline().decode()
				print("[BASESTATION]", logmessage)


			elif packetType == REM_BaseTypes.PACKET_TYPE_REM_ROBOT_LOG:
				logmessage = serial_connection.readline().decode()
				print("[BOT]", logmessage)

			else:
				print(f"Error : Unhandled packet with type {packetType}")
				print(serial_connection.readline().decode())


	except serial.SerialException as se:
		print("SerialException", se)
		serial_connection = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
