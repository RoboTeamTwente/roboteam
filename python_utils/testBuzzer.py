import time
import math
import utils
import serial
import numpy as np
import re
import argparse
import shutil
import multiprocessing
import getch
import sys

import roboteam_embedded_messages.python.BaseTypes as BaseTypes
from roboteam_embedded_messages.python.RobotBuzzer import RobotBuzzer

basestation = None
buzzPacket = RobotBuzzer()

lastWritten = time.time()
packetHz = 60

# q w d | f y j | k | j
# a w c

notes = {
	"q" : 220, # A3
	"a" : 233, # AS3
	"z" : 247, # B3
	"w" : 262, # C4
	"s" : 277, # CS4
	"x" : 294, # D4
	"e" : 311, # DS4
	"d" : 330, # E4
	"c" : 349, # F4
	"r" : 370, # FS4
	"f" : 392, # G4
	"v" : 415, # GS4
	"t" : 440, # A4
	"g" : 466, # AS4
	"b" : 494, # B4
	"y" : 523, # C5
	"h" : 554, # CS5
	"n" : 587, # D5
	"u" : 622, # DS5
	"j" : 659, # E5
	"m" : 698, # F5
	"i" : 740, # FS5
	"k" : 784, # G5
	"o" : 831, # GS5
	"l" : 880, # A5
	"p" : 932, # AS5
}


try:
	if len(sys.argv) != 2:
		raise Exception("Error : Invalid number of arguments. Expected id")
	
	robotId = int(sys.argv[1])
	if robotId < 0 or 15 < robotId:
		raise Exception("Error : Invalid robot id %d. Robot id should be between 0 and 15" % robotId)
except Exception as e:
	print(e)
	print("Error : Run script with \"python testBuzzer.py id\"")
	exit()


buzzPacket.header = BaseTypes.PACKET_TYPE_REM_ROBOT_BUZZER
buzzPacket.remVersion = BaseTypes.LOCAL_REM_VERSION
buzzPacket.id = robotId


def read_input(note):
	while True:
		char = getch.getch()
		if char in notes:
			note.value = notes[char]

note = multiprocessing.Value('i', 0)
inputThread = multiprocessing.Process(target=read_input, args=(note,))
inputThread.start()


while True:
	# Open basestation with the basestation
	if basestation is None or not basestation.isOpen():
		basestation = utils.openContinuous(timeout=0.001)

	try:
		# Continuously read and print messages from the basestation
		while True:

			# Run at 60fps
			if 1./packetHz <= time.time() - lastWritten:
				lastWritten = time.time()

				if note.value != 0:
					buzzPacket.period = note.value
					buzzPacket.duration = 1
					basestation.write(buzzPacket.encode())
					note.value = 0

			# Read feedback packets coming from the robot
			packet_type = basestation.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			if packetType == BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG:
				msg = basestation.readline().decode()
				print("[LOG]", msg, end="")

	except serial.SerialException as se:
		print("SerialException", se)
		basestation = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
