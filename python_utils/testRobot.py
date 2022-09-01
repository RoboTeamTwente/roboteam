import time
from datetime import datetime
import math
from inspect import getmembers
import serial
import numpy as np
import re
import argparse
import sys 
import shutil
import multiprocessing

import utils
from REMParser import REMParser

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from roboteam_embedded_messages.python.REM_RobotGetPIDGains import REM_RobotGetPIDGains
from roboteam_embedded_messages.python.REM_RobotSetPIDGains import REM_RobotSetPIDGains
from roboteam_embedded_messages.python.REM_RobotPIDGains import REM_RobotPIDGains
from roboteam_embedded_messages.python.REM_RobotLog import REM_RobotLog
from roboteam_embedded_messages.python.REM_BasestationLog import REM_BasestationLog
from roboteam_embedded_messages.python.REM_BasestationGetConfiguration import REM_BasestationGetConfiguration
from roboteam_embedded_messages.python.REM_BasestationSetConfiguration import REM_BasestationSetConfiguration
from roboteam_embedded_messages.python.REM_BasestationConfiguration import REM_BasestationConfiguration

robotStateInfoFile = open(f"logs/robotStateInfo_{int(time.time())}.csv", "w+")
robotCommandFile = open(f"logs/robotCommand_{int(time.time())}.csv", "w+")
robotFeedbackFile = open(f"logs/robotFeedback_{int(time.time())}.csv", "w+")

robotStateInfo = REM_RobotStateInfo()
robotFeedback = REM_RobotFeedback()

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

def normalize_angle(angle):
	pi2 = 2*math.pi
	# reduce the angle  
	angle = angle % (pi2)
	# force it to be the positive remainder, so that 0 <= angle < 360  
	angle = (angle + pi2) % pi2
	# force into the minimum absolute value residue class, so that -180 < angle <= 180  
	if (angle > math.pi): angle -= pi2
	return angle

testsAvailable = ["nothing", "full", "kicker-reflect", "kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "forward-rotate", "getpid", "angular-velocity", "circle", "raised-cosine"]

# Parse input arguments 
try:
	if len(sys.argv) != 3:
		raise Exception("Error : Invalid number of arguments. Expected id and test")
	
	robot_id = int(sys.argv[1])
	if robot_id < 0 or 15 < robot_id:
		raise Exception("Error : Invalid robot id %d. Robot id should be between 0 and 15" % robot_id)
	
	test = sys.argv[2]
	if test not in testsAvailable:
		raise Exception("Error : Unknown test %s. Choose a test : %s" % (test, ", ".join(testsAvailable)))
except Exception as e:
	print(e)
	print("Error : Run script with \"python testRobot.py id test\"")
	exit()


basestation = None

tick_counter = 0
periodLength = 300
packetHz = 60

robotConnected = True

doFullTest = test == "full"
testIndex = 2

# stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF525750877267181714-if02"
stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF544852707267223637-if02"

def createSetPIDCommand(robot_id, PbodyX = 0.2, IbodyX = 0.0, DbodyX = 0.0, PbodyY = 0.3, IbodyY = 0.0, DbodyY = 0.0, PbodyW = 0.25, IbodyW = 5.0, DbodyW = 0.0, PbodyYaw = 20.0, IbodyYaw = 5.0, DbodyYaw = 0.0, Pwheels = 2.0, Iwheels = 0.0, Dwheels = 0.0): # Change the default values if the robot PIDs change
	# Create new empty setPID command
	setPID = REM_RobotSetPIDGains()
	setPID.header = BaseTypes.PACKET_TYPE_REM_ROBOT_SET_PIDGAINS
	setPID.remVersion = BaseTypes.LOCAL_REM_VERSION
	setPID.id = robot_id
	
	# Set the PID gains
	setPID.PbodyX = PbodyX
	setPID.IbodyX = IbodyX
	setPID.DbodyX = DbodyX
	
	setPID.PbodyY = PbodyY
	setPID.IbodyY = IbodyY
	setPID.DbodyY = DbodyY
	
	setPID.PbodyW = PbodyW
	setPID.IbodyW = IbodyW
	setPID.DbodyW = DbodyW
	
	setPID.PbodyYaw = PbodyYaw
	setPID.IbodyYaw = IbodyYaw
	setPID.DbodyYaw = DbodyYaw
	
	setPID.Pwheels = Pwheels
	setPID.Iwheels = Iwheels
	setPID.Dwheels = Dwheels
	
	return setPID
	
	

def createRobotCommand(robot_id, test, tick_counter, period_fraction):
	log = ""

	# Seperate test
	if test == "getpid":
		if period_fraction == 0:
			robotGetPIDGains = REM_RobotGetPIDGains()
			robotGetPIDGains.header = BaseTypes.PACKET_TYPE_REM_ROBOT_GET_PIDGAINS
			robotGetPIDGains.remVersion = BaseTypes.LOCAL_REM_VERSION
			robotGetPIDGains.id = robot_id
			return robotGetPIDGains, log

	# Create new empty robot command
	cmd = REM_RobotCommand()
	cmd.header = BaseTypes.PACKET_TYPE_REM_ROBOT_COMMAND
	cmd.remVersion = BaseTypes.LOCAL_REM_VERSION
	cmd.id = robot_id	
	cmd.messageId = tick_counter
	
	counter = 0
	beta = 0.5
	T = 1
	direction = 1
	
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
			cmd.kickChipPower = 1.5

	if test == "dribbler":
		cmd.dribbler = period_fraction
		log = "speed = %.2f" % cmd.dribbler

	if test == "rotate":
		cmd.useAbsoluteAngle = 1
		cmd.angle = -math.pi + 2 * math.pi * ((period_fraction*5 + 0.5) % 1)
		log = "angle = %+.3f" % cmd.angle

	if test == "forward" or test == "sideways":
		if period_fraction == 1:
			counter += 1
		cmd.useAbsoluteAngle = 1
		cmd.rho = 0.3 - 0.3 * math.cos( 4 * math.pi * period_fraction )
		if 0.5 < period_fraction : cmd.theta = -math.pi
		log = "rho = %+.3f theta = %+.3f" % (cmd.rho, cmd.theta)

	if test == "sideways":
		cmd.useAbsoluteAngle = 1
		cmd.angle = math.pi / 2
	
	if test == "circle":
		cmd.useAbsoluteAngle = 1
		cmd.rho = 1
		cmd.theta = period_fraction * 2*math.pi - math.pi

	if test == "rotate-discrete":
		cmd.useAbsoluteAngle = 1
		if period_fraction <=  1.: cmd.angle = math.pi/2
		if period_fraction <= .75: cmd.angle = -math.pi
		if period_fraction <= .50: cmd.angle = -math.pi/2
		if period_fraction <= .25: cmd.angle = 0
		log = "angle = %+.3f" % cmd.angle

	if test == "forward-rotate":
		cmd.useAbsoluteAngle = 0
		cmd.rho = 0.5 - 0.5 * math.cos( 4 * math.pi * period_fraction )
		if 0.5 < period_fraction : cmd.theta = -math.pi
		#cmd.angle = -math.pi + 2 * math.pi * ((period_fraction + 0.5) % 1)
		cmd.angularVelocity = math.pi/3 # set useAbsoluteAngle to 0 to use this
		log = "rho = %+.3f theta = %+.3f angle = %+.3f" % (cmd.rho, cmd.theta, cmd.angle)
		
	if test == "angular-velocity":
		cmd.angularVelocity = math.pi
		log = "rateOfTurn = %+.3f" % robotStateInfo.rateOfTurn

	return cmd, log

# parser = REMParser(basestation)
# parser.parseFile("out.bin")
# print(len(parser.packet_buffer))
# while parser.hasPackets():
# 	packet = parser.getNextPacket()
# exit()


while True:
	try:
		# Loop control
		last_tick_time = time.time()

		latest_packets = {}
		last_robotfeedback_time = 0
		last_robotcommand_time = 0
		last_robotstateinfo_time = 0
		last_basestation_log = ""
		parser = None
		# Visualisation
		image_vis = np.zeros((500, 500, 3), dtype=float)
		wheel_speeds_avg = np.zeros(4)
		rate_of_turn_avg = 0
		
		# Open basestation
		if basestation is None or not basestation.isOpen():
			basestation = utils.openContinuous(timeout=0.01)
			print("Basestation opened")

		# Open writer / parser
		if parser is None and basestation is not None:
			datetime_str = datetime.now().strftime("%Y%m%d_%H%M%S")
			parser = REMParser(basestation, output_file=f"log_{datetime_str}.bin")

		
		# Create and send new PID gains
		# Comment these lines out if you're not tuning PIDs
		#setPID = createSetPIDCommand(robot_id, PbodyW = 0.25, IbodyW = 5.0) #put PID gains as arguments to change them (unchanged keep default value)
		#setPID_encoded = setPID.encode()
		#basestation.write(setPID_encoded)
		#parser.writeBytes(setPID_encoded)

		# Continuously write -> read -> visualise
		while True:

			tick_required = 1./packetHz <= time.time() - last_tick_time

			# ========== WRITING ========== #
			if tick_required:

				# Timing stuff
				last_tick_time += 1./packetHz
				tick_counter += 1
				period = tick_counter % periodLength
				period_fraction = period / periodLength

				# Check connection with robot
				robotConnected = last_robotcommand_time - last_robotfeedback_time < 0.5

				# Update test if needed
				if doFullTest and period == 0:
					testIndex = (testIndex + 1) % len(testsAvailable)
					if testIndex == 0: testIndex = 2
					test = testsAvailable[testIndex]

				# Create and send new robot command
				cmd, cmd_log = createRobotCommand(robot_id, test, tick_counter, period_fraction)
				cmd_encoded = cmd.encode()
				basestation.write(cmd_encoded)
				parser.writeBytes(cmd_encoded)
				last_robotcommand_time = time.time()
				
				# Write packet info to files (used in plotPID.py) 
				

				# if period == 0:
				# 	cmd = REM_BasestationGetConfiguration()
				# 	cmd.header = BaseTypes.PACKET_TYPE_REM_BASESTATION_GET_CONFIGURATION
				# 	cmd.remVersion = BaseTypes.LOCAL_REM_VERSION
				# 	basestation.write(cmd.encode())

				# Logging
				bar = drawProgressBar(period_fraction)
				if not robotConnected:
					print(" Receiving no feedback!", end="")
				print(f" {robot_id} - {test} {bar} {cmd_log} | {last_basestation_log} ", end=" "*23 + "\r")



			# ========== READING ========== # 
			parser.read() # Read all available bytes
			parser.process() # Convert all read bytes into packets

			# Handle and store all new packets
			while parser.hasPackets():
				packet = parser.getNextPacket()
				latest_packets[type(packet)] = packet


			if REM_BasestationLog in latest_packets and latest_packets[REM_BasestationLog] is not None:
				last_basestation_log = latest_packets[REM_BasestationLog].message
				latest_packets[REM_BasestationLog] = None
				if last_basestation_log[-1] == '\n': last_basestation_log = last_basestation_log[:-1]



			# ========== VISUALISING ========== #

			# Break if cv2 is not imported
			if not cv2_available: continue

			# Draw robot on the image
			s = 101.2
			cv2.line(image_vis, (int(250-s/2), 250-73), (int(250+s/2), 250-73), (255,255,255),2)
			cv2.ellipse(image_vis, (250, 250), (90, 90), -90, 35, 325, (255,255,255), 2)			

			### Draw information received from the RobotFeedback packet
			if REM_RobotFeedback in latest_packets and latest_packets[REM_RobotFeedback] is not None:
				robotFeedback = latest_packets[REM_RobotFeedback]
				latest_packets[REM_RobotFeedback] = None
				

				# Ballsensor
				if robotFeedback.ballSensorWorking:
					cv2.line(image_vis, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 1, 0),2)
					if robotFeedback.hasBall:
						cv2.circle(image_vis, (250+int(73*robotFeedback.ballPos), 250-90), 10, (0, 0.4, 1), -1)
				else:
					cv2.line(image_vis, (int(250-s/2), 250-73-5), (int(250+s/2), 250-73-5), (0, 0, 1),2)

				length = int(robotFeedback.rho * 500)
				px, py = rotate((250, 250), (250, 250+length), robotFeedback.theta)
				cv2.line(image_vis, (250,250), (int(px), int(py)), (1, 0, 0), 8)

				dBm = -robotFeedback.rssi/2
				cv2.rectangle(image_vis, (10, 10), (210, 20), (100, 0, 0), 2)
				cv2.rectangle(image_vis, (10, 10), (10 + int(200*(1 - dBm/-80)), 20), (0, 255, 0), -1)

			### Draw information received from the RobotStateInfo packet
			if REM_RobotStateInfo in latest_packets and latest_packets[REM_RobotStateInfo] is not None:
				robotStateInfo = latest_packets[REM_RobotStateInfo]
				latest_packets[REM_RobotStateInfo] = None
				last_robotstateinfo_time = time.time()
				robotStateInfoFile.write(f"{last_robotstateinfo_time} {robotStateInfo.xsensAcc1} {robotStateInfo.xsensAcc2} {robotStateInfo.xsensYaw} {robotStateInfo.rateOfTurn} 					{robotStateInfo.wheelSpeed1} {robotStateInfo.wheelSpeed2} {robotStateInfo.wheelSpeed3} {robotStateInfo.wheelSpeed4} {robotStateInfo.dribbleSpeed} {robotStateInfo.filteredDribbleSpeed} {robotStateInfo.dribblespeedBeforeGotBall	} {robotStateInfo.bodyXIntegral} 				{robotStateInfo.bodyYIntegral} {robotStateInfo.bodyWIntegral} {robotStateInfo.bodyYawIntegral} {robotStateInfo.wheel1Integral} {robotStateInfo.wheel2Integral} 					{robotStateInfo.wheel3Integral} {robotStateInfo.wheel4Integral} \n")
				robotStateInfoFile.flush()
				last_robotfeedback_time = time.time()
				robotFeedbackFile.write(f"{last_robotfeedback_time} {robotFeedback.batteryLevel} {robotFeedback.XsensCalibrated} {robotFeedback.ballSensorWorking} 							{robotFeedback.ballSensorSeesBall} {robotFeedback.dribblerSeesBall} {robotFeedback.capacitorCharged} {robotFeedback.ballPos} {robotFeedback.rho} {robotFeedback.theta} {robotFeedback.angle} 								{robotFeedback.wheelLocked} {robotFeedback.wheelBraking} {robotFeedback.rssi} \n")
				robotFeedbackFile.flush()
				robotCommandFile.write(f"{last_robotcommand_time} {cmd.doKick} {cmd.doChip} {cmd.kickAtAngle} {cmd.doForce} {cmd.useCameraAngle} {cmd.rho} {cmd.theta} {cmd.angle} {cmd.angularVelocity} 					{cmd.cameraAngle} {cmd.dribbler} {cmd.kickChipPower} {cmd.useAbsoluteAngle} \n")
				robotCommandFile.flush()

				# XSens yaw
				px, py = rotate((250, 250), (250, 150), -robotStateInfo.xsensYaw)
				cv2.line(image_vis, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
				cv2.circle(image_vis, (int(px), int(py)), 5, (1, 1, 1), -1)

				# Commanded yaw
				px, py = rotate((250, 250), (250, 150), -cmd.angle)
				cv2.line(image_vis, (250, 250), (int(px), int(py)), (0, 1, 0), 1)
				cv2.circle(image_vis, (int(px), int(py)), 5, (0, 1, 0), -1)
				
				# XSens rate of turn
				rate_of_turn_avg = rate_of_turn_avg * 0.99 + robotStateInfo.rateOfTurn * 0.01
				cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5*-rate_of_turn_avg * 180 / math.pi, (1,.45, .5), 12)
				cv2.ellipse(image_vis, (250, 250), (40, 40), -90, 0, 0.5*-robotStateInfo.rateOfTurn * 180 / math.pi, (1, 1, 1), 4)
				
				# Wheel speeds
				wheel_speeds = np.array([robotStateInfo.wheelSpeed1, robotStateInfo.wheelSpeed2, robotStateInfo.wheelSpeed3, robotStateInfo.wheelSpeed4])
				wheel_speeds_exp = np.log(np.abs(wheel_speeds))
				wheel_speeds_exp = np.clip(wheel_speeds_exp, 0, None)
				wheel_speeds_exp = wheel_speeds_exp * .25 * np.sign(wheel_speeds)
				wheel_speeds_avg = wheel_speeds_avg * 0.99 + wheel_speeds_exp * 0.01

				# XSens wheel speed 1
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_avg[0] * 80), -30 * np.pi / 180.)
				cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 170), (330, 170 - wheel_speeds_exp[0] * 80), -30 * np.pi / 180.)
				cv2.line(image_vis, (330, 170), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 2
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_avg[1] * 80), 60 * np.pi / 180.)
				cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((330, 330), (330, 330 - wheel_speeds_exp[1] * 80), 60 * np.pi / 180.)
				cv2.line(image_vis, (330, 330), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 3
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_avg[2] * 80), -60 * np.pi / 180.)
				cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 330), (170, 330 + wheel_speeds_exp[2] * 80), -60 * np.pi / 180.)
				cv2.line(image_vis, (170, 330), (int(rx), int(ry)), (1, 1, 1), 4)
				# XSens wheel speed 4
				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_avg[3] * 80), 30 * np.pi / 180.)
				cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (.15, .15, 1), 10)
				rx, ry = rotate((170, 170), (170, 170 + wheel_speeds_exp[3] * 80), 30 * np.pi / 180.)
				cv2.line(image_vis, (170, 170), (int(rx), int(ry)), (1, 1, 1), 4)
			

			if REM_BasestationConfiguration in latest_packets and latest_packets[REM_BasestationConfiguration] is not None:
				packet = latest_packets[REM_BasestationConfiguration]
				print("\n[BS_CONF] Channel", "yellow" if packet.channel == 0 else "blue")
				latest_packets[REM_BasestationConfiguration] = None

			if tick_required:
				cv2.imshow("Press esc to quit", image_vis)
				if cv2.waitKey(1) == 27: exit()
				image_vis *= 0.7

			# for packet_type in latest_packets.keys():
			# 	if latest_packets[packet_type] is not None:
			# 		print(f"Unhandled packet {packet_type.__name__}")
			# 		latest_packets[packet_type] = None

			


	except serial.SerialException as se:
		print("SerialException", se)
		basestation = None
		last_basestation_log = ""
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
		raise e
