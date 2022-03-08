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

try:
	from rem import rem
except ImportError:
	print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
	print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
	print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
	exit()

def printPacket(rc):
	maxLength = max([len(k) for k, v in getmembers(rc)])
	title = re.findall(r"_(\w+) ", str(rc))[0]
	
	lines = [("┌─ %s "%title) + ("─"*100)[:maxLength*2+2-len(title)] + "┐" ]	
	lines += [ "│ %s : %s │" % ( k.rjust(maxLength) , str(v)[:6].ljust(maxLength) ) for k, v in getmembers(rc) ]
	lines += [ "└" + ("─"*(maxLength*2+5)) + "┘"]
	print("\n"*20 + "\n".join(lines))

def drawProgressBar(progress):
	cols = min(40, shutil.get_terminal_size((80, 20)).columns)
	filled = int(cols*progress)
	string = "["
	string += "*" * filled
	string += " " * (cols - filled)
	string += "]"
	return string

lastWritten = time.time()
tickCounter = 0
periodLength = 300

packetsReceived = 0
robotConnected = True

connection = None

# connection = serial.Serial(port="/dev/ttyACM0", timeout=0.001, baudrate=115200)
total_bytes_received = 0
while True:
	# Open basestation with the basestation

	if connection is None or not connection.isOpen():
		connection = utils.openContinuous(timeout = 0.1)

	try:
		bytes_received = 0
		# Continuously read and print messages from the basestation
		while True:

			# Read feedback packets coming from the robot
			packet_type = connection.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			if packetType == rem.lib.PACKET_TYPE_REM_ROBOT_COMMAND:
				packet = packet_type + connection.read(rem.lib.PACKET_SIZE_REM_ROBOT_COMMAND - 1)
				payload = rem.ffi.new("RobotCommandPayload*")
				payload.payload = packet

				cmd = rem.ffi.new("RobotCommand*")
				rem.lib.decodeRobotCommand(cmd, payload)
				printPacket(cmd)

			if packetType == rem.lib.PACKET_TYPE_REM_BASESTATION_LOG:
				line = connection.readline().decode()
				bytes_received += len(line) + 1
				total_bytes_received += len(line) + 1
				print(f"{total_bytes_received} {bytes_received} [LOG] {line}", end="")
				continue

			if packetType == rem.lib.PACKET_TYPE_REM_BASESTATION_STATISTICS:
				print("[Statistics]")
				packet = packet_type + connection.read(rem.lib.PACKET_SIZE_REM_BASESTATION_STATISTICS - 1)
				print(type(packet), len(packet), packet[1], packet[2], packet[3], packet[4])
				# payload = rem.ffi.new("RobotCommandPayload*")
				# payload.payload = packet

				# cmd = rem.ffi.new("RobotCommand*")
				# rem.lib.decodeRobotCommand(cmd, payload)
				# printPacket(cmd)

	except serial.SerialException as se:
		print("SerialException", se)
		connection = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		# ser = None