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
import getch

try:
	import cv2
	cv2_available = True
except:
	print("Warning! Could not import cv2. Can't visualize.")
	cv2_available = False

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

def rotate(origin, point, angle):
    ox, oy = origin
    px, py = point

    qx = ox + math.cos(angle) * (px - ox) - math.sin(angle) * (py - oy)
    qy = oy + math.sin(angle) * (px - ox) + math.cos(angle) * (py - oy)
    return qx, qy

testsAvailable = ["nothing", "full", "kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "forward-rotate"]

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
commandPayload = rem.ffi.new("RobotCommandPayload*")

feedbackPayload = rem.ffi.new("RobotFeedbackPayload*")
feedback = rem.ffi.new("RobotFeedback*")
feedbackTimestamp = 0

stateInfoPayload = rem.ffi.new("RobotStateInfoPayload*")
stateInfo = rem.ffi.new("RobotStateInfo*");
stateInfoTimestamp = 0

wheel_speeds_avg = np.zeros(4)
rate_of_turn_avg = 0

lastWritten = time.time()
tickCounter = 0
periodLength = 300
packetHz = 60

totalCommandsSent = 0
totalFeedbackReceived = 0
robotConnected = True

lastBasestationLog = ""


doFullTest = test == "full"
testIndex = 2

# t = time.time()
# logfile = open(f"logfile_{t}.txt", "w")

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
				cmd = rem.ffi.new("RobotCommand*")
				cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
				cmd.id = robotId

				# All tests
				log = ""

				if True: # This if-statement is just here so that I can easily collapse this large amount of code
					if test == "nothing":
						cmd.rho = 0
						cmd.theta = 0
						cmd.angle = 0	

					if test == "kicker" or test == "chipper":
						if period == 0:
							if test == "kicker"  : cmd.doKick = True
							if test == "chipper" : cmd.doChip = True
							cmd.doForce = True
							cmd.kickChipPower = 2

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
				rem.lib.encodeRobotCommand(commandPayload, cmd)
				basestation.write(commandPayload.payload)
				totalCommandsSent += 1


			# Read feedback pacstateInfoPayloadkets coming from the robot
			packet_type = basestation.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]


			if packetType == rem.lib.PACKET_TYPE_ROBOT_FEEDBACK:
				feedbackTimestamp = time.time()
				packet = packet_type + basestation.read(rem.lib.PACKET_SIZE_ROBOT_FEEDBACK - 1)
				feedbackPayload.payload = packet
				rem.lib.decodeRobotFeedback(feedback, feedbackPayload)

				if feedback.id != robotId:
					print("Error : Received feedback from robot %d ???" % receivedId)
				totalFeedbackReceived += 1


			if packetType == rem.lib.PACKET_TYPE_ROBOT_STATE_INFO:
				stateInfoTimestamp = time.time()
				packet = packet_type + basestation.read(rem.lib.PACKET_SIZE_ROBOT_STATE_INFO - 1)
				stateInfoPayload.payload = packet
				rem.lib.decodeRobotStateInfo(stateInfo, stateInfoPayload)

			if packetType == rem.lib.PACKET_TYPE_BASESTATION_STATISTICS:
				print("[Statistics]")
				packet = packet_type + basestation.read(rem.lib.PACKET_SIZE_BASESTATION_STATISTICS - 1)
				print(type(packet), len(packet), packet[1], packet[2], packet[3], packet[4])


			if packetType == rem.lib.PACKET_TYPE_BASESTATION_LOG:
				logmessage = basestation.readline().decode()
				lastBasestationLog = logmessage[:-1] + " "*20

			# break
			if not cv2_available:
				break

			img *= 0.5

			s = 101.2
			cv2.line(img, (int(250-s/2), 250-73), (int(250+s/2), 250-73), (255,255,255),2)
			cv2.ellipse(img, (250, 250), (90, 90), -90, 35, 325, (255,255,255), 2)			

			if time.time() - feedbackTimestamp < 1:
				# if not feedback.

				if feedback.ballSensorWorking:
					cv2.line(img, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 1, 0),2)
					if feedback.hasBall:
						cv2.circle(img, (250+int(73*feedback.ballPos), 250-90), 10, (0, 0.4, 1), -1)
				else:
					cv2.line(img, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 0, 1),2)

				# print(f"{feedback.rho:.5f}, {feedback.theta:.3f}")

				length = int(feedback.rho * 500)
				px, py = rotate((250, 250), (250, 250+length), feedback.theta)
				cv2.line(img, (250,250), (int(px), int(py)), (1, 0, 0), 8)

			if time.time() - stateInfoTimestamp < 1:

				# XSens yaw
				px, py = rotate((250, 250), (250, 150), -stateInfo.xsensYaw)
				cv2.line(img, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
				cv2.circle(img, (int(px), int(py)), 5, (1, 1, 1), -1)
				
				# XSens rate of turn
				rate_of_turn_avg = rate_of_turn_avg * 0.99 + stateInfo.rateOfTurn * 0.01
				cv2.ellipse(img, (250, 250), (40, 40), -90, 0, 0.5*-rate_of_turn_avg * 180 / math.pi, (1,.45, .5), 12)
				cv2.ellipse(img, (250, 250), (40, 40), -90, 0, 0.5*-stateInfo.rateOfTurn * 180 / math.pi, (1, 1, 1), 4)
				
				# Wheel speeds
				wheel_speeds = np.array([stateInfo.wheelSpeed1, stateInfo.wheelSpeed2, stateInfo.wheelSpeed3, stateInfo.wheelSpeed4])
				wheel_speeds_exp = np.log(np.abs(wheel_speeds))
				wheel_speeds_exp = np.clip(wheel_speeds_exp, 0, None)
				wheel_speeds_exp = wheel_speeds_exp * .25 * np.sign(wheel_speeds)
				wheel_speeds_avg = wheel_speeds_avg * 0.99 + wheel_speeds_exp * 0.01

				# XSens wheel speed 1				
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_avg[0] * 80), -30 * np.pi / 180.)
				cv2.line(img, (330, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_exp[0] * 80), -30 * np.pi / 180.)
				cv2.line(img, (330, 170), (int(rx), int(ry)), (1, 1, 1), 4)
				
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_avg[1] * 80), 60 * np.pi / 180.)
				cv2.line(img, (330, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_exp[1] * 80), 60 * np.pi / 180.)
				cv2.line(img, (330, 330), (int(rx), int(ry)), (1, 1, 1), 4)
				
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_avg[2] * 80), -60 * np.pi / 180.)
				cv2.line(img, (170, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_exp[2] * 80), -60 * np.pi / 180.)
				cv2.line(img, (170, 330), (int(rx), int(ry)), (1, 1, 1), 4)

				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_avg[3] * 80), 30 * np.pi / 180.)
				cv2.line(img, (170, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_exp[3] * 80), 30 * np.pi / 180.)
				cv2.line(img, (170, 170), (int(rx), int(ry)), (1, 1, 1), 4)
				
				# XSens wheel speed 1
				# cv2.ellipse(img, (400, 100), (40, 40), -90, 0, wheel_speeds_avg[0] * 180, (.15,.15, 1), -1)
				# cv2.ellipse(img, (400, 100), (30, 30), -90, 0, wheel_speeds_exp[0] * 180, (1,1,1), -1)
				# # XSens wheel speed 2
				# cv2.ellipse(img, (400, 400), (40, 40), -90, 0, wheel_speeds_avg[1] * 180, (.15,.15, 1), -1)
				# cv2.ellipse(img, (400, 400), (30, 30), -90, 0, wheel_speeds_exp[1] * 180, (1,1,1), -1)
				# # XSens wheel speed 3
				# cv2.ellipse(img, (100, 400), (40, 40), -90, 0, wheel_speeds_avg[2] * 180, (.15,.15, 1), -1)
				# cv2.ellipse(img, (100, 400), (30, 30), -90, 0, wheel_speeds_exp[2] * 180, (1,1,1), -1)
				# # XSens wheel speed 4
				# cv2.ellipse(img, (100, 100), (40, 40), -90, 0, wheel_speeds_avg[3] * 180, (.15,.15, 1), -1)
				# cv2.ellipse(img, (100, 100), (30, 30), -90, 0, wheel_speeds_exp[3] * 180, (1,1,1), -1)

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
		# Res the connection to the basestation
		# ser = None
