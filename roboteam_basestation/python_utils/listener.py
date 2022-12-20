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
import traceback

import roboteam_embedded_messages.python.REM_BaseTypes as REM_BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from roboteam_embedded_messages.python.REM_Log import REM_Log

from REMParser import REMParser

parser = argparse.ArgumentParser()
parser.add_argument('--robotcommand', '-r', help='Print REM_RobotCommand', action='store_true')
parser.add_argument('--robotfeedback', '-f', help='Print REM_RobotFeedback', action='store_true')
parser.add_argument('--robotstateinfo', '-s', help='Print REM_RobotStateInfo', action='store_true')
parser.add_argument('--log', '-l', help='Print REM_Log', action='store_true')

parser.add_argument('--verbose', '-v', help='Print entire packet', action='store_true')
parser.add_argument('--all', help='Print all packets', action='store_true')
parser.print_help()

args = parser.parse_args()
print()

serial_connection = None
parser = None

robotCommand = REM_RobotCommand()
robotFeedback = REM_RobotFeedback()
robotStateInfo = REM_RobotStateInfo()

feedbackTimestamp = 0
stateInfoTimestamp = 0

packet_types_selected = []
if args.all or args.robotcommand:  packet_types_selected.append(REM_RobotCommand)
if args.all or args.robotfeedback: packet_types_selected.append(REM_RobotFeedback)
if args.all or args.robotstateinfo: packet_types_selected.append(REM_RobotStateInfo)
if args.all or args.log: packet_types_selected.append(REM_Log)
print("Listening for the following packet types:", ", ".join([ o.__name__ for o in packet_types_selected]))
print()

if len(packet_types_selected) == 0:
	print("No packet types were selected to listen to.")
	print("Run 'listener.py --all' to listen to all packets")
	exit()


while True:
	# Open serial_connection with the serial_connection
	if serial_connection is None or not serial_connection.isOpen():
		serial_connection = utils.openContinuous(timeout=0.01)
		if parser is not None: parser.device = serial_connection

	if parser is None and serial_connection is not None:
		parser = REMParser(serial_connection)

	try:
		# Continuously read and print messages from the serial_connection
		while True:
			time.sleep(0.005)

			# ========== READING ========== # 
			parser.read() # Read all available bytes
			parser.process() # Convert all read bytes into packets

			while parser.hasPackets():
				packet = parser.getNextPacket()
				if type(packet) not in packet_types_selected: continue

				if args.verbose:
					utils.printCompletePacket(packet)
				else:
					timestamp = str(packet.timestamp).rjust(5)

					sender = str(packet.fromRobotId).rjust(2)
					if packet.fromBS: sender = "BS"
					if packet.fromPC: senders = "PC"

					message = ""							
					if type(packet) == REM_Log:
						message = packet.message.strip()

					print(f"[{timestamp}][{type(packet).__name__}][{sender}] {message}")


	except serial.SerialException as se:
		print("SerialException", se)
		serial_connection = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("\n[Exception]", e)
		serial_connection = None
		# raise e
		# print(traceback.format_exc())
