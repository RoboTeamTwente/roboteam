import time
import math
from inspect import getmembers
import utils
import serial
import numpy as np
import re
import argparse
import shutil
import multiprocessing
import getch

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
	lines += [ "│ %s : %s │" % ( k.rjust(maxLength) , str(v).ljust(maxLength) ) for k, v in getmembers(rc) ]
	lines += [ "└" + ("─"*(maxLength*2+5)) + "┘"]
	print("\n".join(lines))

def drawProgressBar(progress):
	cols = min(40, shutil.get_terminal_size((80, 20)).columns)
	filled = int(cols*progress)
	string = "["
	string += "*" * filled
	string += " " * (cols - filled)
	string += "]"
	return string

basestation = None
cmdPayload = rem.ffi.new("RobotCommandPayload*")
buzzPayload = rem.ffi.new("RobotBuzzerPayload*")

lastWritten = time.time()
tickCounter = 0
periodLength = 100
packetHz = 60

totalCommandsSent = 0
totalFeedbackReceived = 0
robotConnected = True

lastBasestationLog = ""

counter = 0





    # {NF3,0,NG3,0,NF3,0,NF3,NA3,0,NF3,0,ND3,0,NF3,0,NC4,0,NF3,0,NF3,NC4,0,NC4,0,NG3,0,NF3,0,NC4,0,NF4,0,NF3,ND3,0,ND3,NC3,0,NG3,0,NF3},
notes = {
	"d" : 294,
	"f" : 349,
	"g" : 392,
	"c" : 262,
	"a" : 440,
	"5" : 587,
	"3" : 220,
	"e" : 330,
	"s" : 466
}



# notes = {
# 	"c" : 523,
# 	"d" : 587,
# 	"e" : 659,
# 	"f" : 698,
# 	"g" : 784,
# 	"a" : 880,
# 	"b" : 988,
# 	"x" : 1047
# }

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

				cmd = rem.ffi.new("RobotCommand*")
				cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
				cmd.id = 3
				rem.lib.encodeRobotCommand(cmdPayload, cmd)
				basestation.write(cmdPayload.payload)

				if note.value != 0:
					print("note", note.value)
					cmd = rem.ffi.new("RobotBuzzer*")
					cmd.header = rem.lib.PACKET_TYPE_ROBOT_BUZZER
					cmd.id = 3
					cmd.period = note.value
					cmd.duration = 1
					rem.lib.encodeRobotBuzzer(buzzPayload, cmd)
					basestation.write(buzzPayload.payload)
					note.value = 0
				# if counter % 10 == 0:
				# 	for i in cmdPayload.payload:
				# 		print(i, end=" ")
				# 	print()
				# for i in buzzPayload.payload:
				# 	print(i, end=" ")
				# print()


				# if counter % 10 == 0:
				# 	for i in cmdPayload.payload:
				# 		print(hex(i), end=" ")
				# 	print()
				# for i in buzzPayload.payload:
				# 	print(hex(i), end=" ")
				# print()


				counter += 1

			# Read feedback packets coming from the robot
			packet_type = basestation.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			# if packetType == rem.lib.PACKET_TYPE_ROBOT_FEEDBACK:
			# 	print("Feedback")

			# if packetType == rem.lib.PACKET_TYPE_BASESTATION_STATISTICS:
			# 	print("Statistics")

			if packetType == rem.lib.PACKET_TYPE_BASESTATION_LOG:
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
		# Reset the connection to the basestation
		# ser = None
