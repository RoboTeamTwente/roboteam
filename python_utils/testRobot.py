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

testsAvailable = ["kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "forward-rotate"]

try:
	if len(sys.argv) != 3:
		raise Exception("Error : Invalid number of arguments. Expected id and test")
	
	robotId = int(sys.argv[1])
	if robotId < 0 or 15 < robotId:
		raise Exception("Error : Invalid robot id %d. Robot id should be between 0 and 15" % robotId)
	
	test = sys.argv[2]
	if test not in testsAvailable:
		raise Exception("Error : Unknown test %s. Choose a test : %s" % (test, ", ".join(testsAvailable)))
except Exception as e:
	print(e)
	print("Error : Run script with \"python testRobot.py id test\"")
	exit()

basestation = None
cmdPayload = rem.ffi.new("RobotCommandPayload*")

lastWritten = time.time()
tickCounter = 0
periodLength = 500
packetHz = 60

totalCommandsSent = 0
totalFeedbackReceived = 0
robotConnected = True

lastBasestationLog = ""

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
				tickCounter += 1
				period = tickCounter % periodLength
				periodFraction = period / periodLength

				# Robot connectivity
				if period == 0:
					totalCommandsSent = 0
					totalFeedbackReceived = 0
				if 0.5 < periodFraction:
					robotConnected = True
					# If less than half of feedback packets received
					if totalFeedbackReceived < 0.5 * packetHz * periodFraction:
						robotConnected = False

				# Create new empty robot command
				cmd = rem.ffi.new("RobotCommand*")
				cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
				cmd.id = robotId

				# All tests
				log = ""

				if test == "kicker" or test == "chipper":
					if period == 0:
						if test == "kicker"  : cmd.doKick = True
						if test == "chipper" : cmd.doChip = True
						cmd.doForce = True
						cmd.kickChipPower = 3

				if test == "dribbler":
					cmd.dribbler = math.floor(8 * periodFraction)
					log = "speed = %d" % cmd.dribbler

				if test == "rotate":
					cmd.angle = -math.pi + 2 * math.pi * ((periodFraction*4 + 0.5) % 1)
					log = "angle = %+.3f" % cmd.angle

				if test == "forward" or test == "sideways":
					cmd.rho = 0.5 - 0.5 * math.cos( 4 * math.pi * periodFraction )
					if 0.5 < periodFraction : cmd.theta = -math.pi
					log = "rho = %+.3f theta = %+.3f" % (cmd.rho, cmd.theta)

				if test == "sideways":
					cmd.angle = math.pi / 2

				if test == "rotate-discrete":
					if periodFraction <=  1.: cmd.angle = math.pi/2
					if periodFraction <= .75: cmd.angle = -math.pi
					if periodFraction <= .50: cmd.angle = -math.pi/2
					if periodFraction <= .25: cmd.angle = 0
					log = "angle = %+.3f" % cmd.angle

				if test == "forward-rotate":
					cmd.rho = 0.5 - 0.5 * math.cos( 4 * math.pi * periodFraction )
					if 0.5 < periodFraction : cmd.theta = -math.pi
					cmd.angle = -math.pi + 2 * math.pi * ((periodFraction + 0.5) % 1)
					log = "rho = %+.3f theta = %+.3f angle = %+.3f" % (cmd.rho, cmd.theta, cmd.angle)

				# Logging
				bar = drawProgressBar(periodFraction)
				if not robotConnected:
					print(" Receiving no feedback!", end="")
				tcs, tfr = totalCommandsSent, totalFeedbackReceived
				print(f" {robotId} - {test} {bar} {log} | "
					f"Sent:{tcs} Rcvd:{tfr} ({100*(tfr+1)/(tcs+1):.0f}%) | "
					f"{lastBasestationLog}", end="\r")

				# Send command
				rem.lib.encodeRobotCommand(cmdPayload, cmd)
				basestation.write(cmdPayload.payload)
				totalCommandsSent += 1


			# Read feedback packets coming from the robot
			packet_type = basestation.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			if packetType == rem.lib.PACKET_TYPE_ROBOT_FEEDBACK:
				packet = packet_type + basestation.read(rem.lib.PACKET_SIZE_ROBOT_FEEDBACK - 1)
				payload = rem.ffi.new("RobotFeedbackPayload*")
				payload.payload = packet

				receivedId = rem.lib.RobotFeedback_get_id(payload)
				if receivedId != robotId:
					print("Error : Received feedback from robot %d ???" % receivedId)
					exit()
				totalFeedbackReceived += 1

			if packetType == rem.lib.PACKET_TYPE_BASESTATION_STATISTICS:
				print("[Statistics]")
				packet = packet_type + basestation.read(rem.lib.PACKET_SIZE_BASESTATION_STATISTICS - 1)
				print(type(packet), len(packet), packet[1], packet[2], packet[3], packet[4])

			if packetType == rem.lib.PACKET_TYPE_BASESTATION_LOG:
				lastBasestationLog = basestation.readline().decode()[:-1] + " "*20

	except serial.SerialException as se:
		print("SerialException", se)
		ser = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		# ser = None
