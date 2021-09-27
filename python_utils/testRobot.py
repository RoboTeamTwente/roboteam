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

import roboteam_embedded_messages.python.BaseTypes as BaseTypes
from roboteam_embedded_messages.python.RobotCommand import RobotCommand
from roboteam_embedded_messages.python.RobotFeedback import RobotFeedback
from roboteam_embedded_messages.python.RobotStateInfo import RobotStateInfo



robotStateInfoFile = open(f"robotStateInfo_{int(time.time())}.csv", "w")
robotFeedbackFile = open(f"robotFeedback_{int(time.time())}.csv", "w")




try:
	import cv2
	cv2_available = True
except:
	print("Warning! Could not import cv2. Can't visualize.")
	cv2_available = False

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

def rotate(origin, point, angle):
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

testsAvailable = ["nothing", "full", "kicker-reflect", "kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "forward-rotate"]

# Parse input arguments 
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


### Needed for visualizing RobotStateInfo
img = np.zeros((500, 500, 3), dtype=np.float)

basestation = None

robotCommand = RobotCommand()
robotFeedback = RobotFeedback()
robotStateInfo = RobotStateInfo()

feedbackTimestamp = 0
stateInfoTimestamp = 0

wheel_speeds_avg = np.zeros(4)
rate_of_turn_avg = 0

lastWritten = time.time()
tickCounter = 0
periodLength = 500
packetHz = 60

totalCommandsSent = 0
totalFeedbackReceived = 0
robotConnected = True

lastBasestationLog = ""

doFullTest = test == "full"
testIndex = 2

# stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF525750877267181714-if02"
stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF544852707267223637-if02"

while True:
	# Open basestation with the basestation
	if basestation is None or not basestation.isOpen():
		basestation = utils.openContinuous(timeout=0.001)

	try:
		# Continuously read and print messages from the basestation
		while True:

			# Run at 60fps
			if 1./packetHz <= time.time() - lastWritten:

				# Timing stuff
				lastWritten += 1./packetHz
				tickCounter += 1
				period = tickCounter % periodLength
				periodFraction = period / periodLength

				# Check connection with robot
				if 0.5 < periodFraction:
					robotConnected = True
					# If less than half of feedback packets received
					if totalFeedbackReceived < 0.5 * packetHz * periodFraction:
						robotConnected = False

				# Update test if needed
				if doFullTest and period == 0:
					testIndex = (testIndex + 1) % len(testsAvailable)
					if testIndex == 0: testIndex = 2
					test = testsAvailable[testIndex]

				# Create new empty robot command
				cmd = RobotCommand()
				cmd.header = BaseTypes.PACKET_TYPE_ROBOT_COMMAND
				cmd.remVersion = BaseTypes.LOCAL_REM_VERSION
				cmd.id = robotId

				# All tests
				log = ""

				if True: # This if-statement is just here so that I can easily collapse this large amount of code
					if test == "nothing":
						cmd.rho = 0
						cmd.theta = 0
						cmd.angle = 0	

					if test == "kicker-reflect":
						cmd.doKick = True
						cmd.kickChipPower = 0.2

					if test == "kicker" or test == "chipper":
						if period == 0:
							if test == "kicker"  : cmd.doKick = True
							if test == "chipper" : cmd.doChip = True
							cmd.doForce = True
							cmd.kickChipPower = 0.2

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
					f"{lastBasestationLog}", end="\r")

				# Send command
				if test != "nothing":
					basestation.write( cmd.encode() )
					totalCommandsSent += 1












			### Read any packets coming from the basestation
			# Read packet type
			packet_type = basestation.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]

			# Parse packet based on packet type
			if packetType == BaseTypes.PACKET_TYPE_ROBOT_FEEDBACK:
				feedbackTimestamp = time.time()
				packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_ROBOT_FEEDBACK - 1)

				if RobotFeedback.get_id(packet) == robotId:
					robotFeedback.decode(packet)
					totalFeedbackReceived += 1
				else:
					print("Error : Received feedback from robot %d ???" % RobotFeedback.get_id(packet))
	
			elif packetType == BaseTypes.PACKET_TYPE_ROBOT_STATE_INFO:
				stateInfoTimestamp = time.time()
				packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_ROBOT_STATE_INFO - 1)

				if RobotStateInfo.get_id(packet) == robotId:
					robotStateInfo.decode(packet)
					robotStateInfoFile.write(f"{stateInfoTimestamp} {robotStateInfo.xsensYaw} {robotStateInfo.wheelSpeed1} {robotStateInfo.wheelSpeed2} {robotStateInfo.wheelSpeed3} {robotStateInfo.wheelSpeed4}\n")
					robotStateInfoFile.flush()

				else:
					print("Error : Received StateInfo from robot %d ???" % RobotFeedback.get_id(packet))

			elif packetType == BaseTypes.PACKET_TYPE_BASESTATION_LOG:
				logmessage = basestation.readline().decode()
				lastBasestationLog = logmessage[:-1] + " "*20

			elif packetType == BaseTypes.PACKET_TYPE_ROBOT_LOG:
				logmessage = basestation.readline().decode()
				print("[BOT]", logmessage)
				# lastBasestationLog = logmessage[:-1] + " "*20
			else:
				print(f"Error : Unhandled packet with type {packetType}")

			# Break if cv2 is not imported
			if not cv2_available:
				break










			img *= 0.5

			s = 101.2
			cv2.line(img, (int(250-s/2), 250-73), (int(250+s/2), 250-73), (255,255,255),2)
			cv2.ellipse(img, (250, 250), (90, 90), -90, 35, 325, (255,255,255), 2)			

			### Draw information received from the RobotFeedback packet
			if time.time() - feedbackTimestamp < 1:
				# Ballsensor
				if robotFeedback.ballSensorWorking:
					cv2.line(img, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 1, 0),2)
					if robotFeedback.hasBall:
						cv2.circle(img, (250+int(73*robotFeedback.ballPos), 250-90), 10, (0, 0.4, 1), -1)
				else:
					cv2.line(img, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 0, 1),2)

				length = int(robotFeedback.rho * 500)
				px, py = rotate((250, 250), (250, 250+length), robotFeedback.theta)
				cv2.line(img, (250,250), (int(px), int(py)), (1, 0, 0), 8)

			### Draw information received from the RobotStateInfo packet
			if time.time() - stateInfoTimestamp < 1:

				# XSens yaw
				px, py = rotate((250, 250), (250, 150), -robotStateInfo.xsensYaw)
				cv2.line(img, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
				cv2.circle(img, (int(px), int(py)), 5, (1, 1, 1), -1)
				
				# XSens rate of turn
				rate_of_turn_avg = rate_of_turn_avg * 0.99 + robotStateInfo.rateOfTurn * 0.01
				cv2.ellipse(img, (250, 250), (40, 40), -90, 0, 0.5*-rate_of_turn_avg * 180 / math.pi, (1,.45, .5), 12)
				cv2.ellipse(img, (250, 250), (40, 40), -90, 0, 0.5*-robotStateInfo.rateOfTurn * 180 / math.pi, (1, 1, 1), 4)
				
				# Wheel speeds
				wheel_speeds = np.array([robotStateInfo.wheelSpeed1, robotStateInfo.wheelSpeed2, robotStateInfo.wheelSpeed3, robotStateInfo.wheelSpeed4])
				wheel_speeds_exp = np.log(np.abs(wheel_speeds))
				wheel_speeds_exp = np.clip(wheel_speeds_exp, 0, None)
				wheel_speeds_exp = wheel_speeds_exp * .25 * np.sign(wheel_speeds)
				wheel_speeds_avg = wheel_speeds_avg * 0.99 + wheel_speeds_exp * 0.01

				# XSens wheel speed 1
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_avg[0] * 80), -30 * np.pi / 180.)
				cv2.line(img, (330, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_exp[0] * 80), -30 * np.pi / 180.)
				cv2.line(img, (330, 170), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 2
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_avg[1] * 80), 60 * np.pi / 180.)
				cv2.line(img, (330, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_exp[1] * 80), 60 * np.pi / 180.)
				cv2.line(img, (330, 330), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 3
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_avg[2] * 80), -60 * np.pi / 180.)
				cv2.line(img, (170, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_exp[2] * 80), -60 * np.pi / 180.)
				cv2.line(img, (170, 330), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 4
				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_avg[3] * 80), 30 * np.pi / 180.)
				cv2.line(img, (170, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_exp[3] * 80), 30 * np.pi / 180.)
				cv2.line(img, (170, 170), (int(rx), int(ry)), (1, 1, 1), 4)
				
			cv2.imshow("img", img)
			if cv2.waitKey(1) == 27: exit()


	except serial.SerialException as se:
		print("SerialException", se)
		basestation = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
