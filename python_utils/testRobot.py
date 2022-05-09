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
from collections import deque

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from roboteam_embedded_messages.python.REM_RobotStateInfo import REM_RobotStateInfo
from roboteam_embedded_messages.python.REM_RobotGetPIDGains import REM_RobotGetPIDGains
from roboteam_embedded_messages.python.REM_RobotPIDGains import REM_RobotPIDGains
from roboteam_embedded_messages.python.REM_RobotPIDGains import REM_RobotPIDGains
from roboteam_embedded_messages.python.REM_RobotPIDGains import REM_RobotPIDGains
from roboteam_embedded_messages.python.REM_RobotLog import REM_RobotLog
from roboteam_embedded_messages.python.REM_BasestationLog import REM_BasestationLog
from roboteam_embedded_messages.python.REM_BasestationGetConfiguration import REM_BasestationGetConfiguration
from roboteam_embedded_messages.python.REM_BasestationSetConfiguration import REM_BasestationSetConfiguration
from roboteam_embedded_messages.python.REM_BasestationConfiguration import REM_BasestationConfiguration

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

testsAvailable = ["nothing", "full", "kicker-reflect", "kicker", "chipper", "dribbler", "rotate", "forward", "sideways", "rotate-discrete", "forward-rotate", "getpid"]

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

img = np.zeros((500, 500, 3), dtype=np.float)

basestation = None

wheel_speeds_avg = np.zeros(4)
rate_of_turn_avg = 0

lastWritten = time.time()
tickCounter = 0
periodLength = 300
packetHz = 60

totalCommandsSent = 0
totalFeedbackReceived = 0
robotConnected = True

doFullTest = test == "full"
testIndex = 2

# stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_0674FF525750877267181714-if02"
stlink_port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066FFF544852707267223637-if02"

def createRobotCommand(robot_id, test, period_fraction):
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
			cmd.kickChipPower = BaseTypes.PACKET_RANGE_REM_ROBOT_COMMAND_KICK_CHIP_POWER_MAX // 2

	if test == "dribbler":
		cmd.dribbler = period_fraction
		log = "speed = %.2f" % cmd.dribbler

	if test == "rotate":
		cmd.angle = -math.pi + 2 * math.pi * ((period_fraction*4 + 0.5) % 1)
		log = "angle = %+.3f" % cmd.angle

	if test == "forward" or test == "sideways":
		cmd.rho = 0.5 - 0.5 * math.cos( 4 * math.pi * period_fraction )
		if 0.5 < period_fraction : cmd.theta = -math.pi
		log = "rho = %+.3f theta = %+.3f" % (cmd.rho, cmd.theta)

	if test == "sideways":
		cmd.angle = math.pi / 2

	if test == "rotate-discrete":
		if period_fraction <=  1.: cmd.angle = math.pi/2
		if period_fraction <= .75: cmd.angle = -math.pi
		if period_fraction <= .50: cmd.angle = -math.pi/2
		if period_fraction <= .25: cmd.angle = 0
		log = "angle = %+.3f" % cmd.angle

	if test == "forward-rotate":
		cmd.rho = 0.5 - 0.5 * math.cos( 4 * math.pi * period_fraction )
		if 0.5 < period_fraction : cmd.theta = -math.pi
		cmd.angle = -math.pi + 2 * math.pi * ((period_fraction + 0.5) % 1)
		log = "rho = %+.3f theta = %+.3f angle = %+.3f" % (cmd.rho, cmd.theta, cmd.angle)

	return cmd, log

class REMReader():
	
	def __init__(self, device):
		print(f"[REMReader] New REMReader for device {device.port}")

		self.device = device
		self.byte_buffer = bytes()
		self.packet_buffer = deque()

	def read(self):

		bytes_in_waiting = self.device.inWaiting()
		if bytes_in_waiting == 0: return
		# print(f"[read] {bytes_in_waiting} bytes in waiting")
		self.byte_buffer += self.device.read(bytes_in_waiting)
		# print(f"Read {bytes_in_waiting} bytes")

	def process(self):
		# No bytes in the buffer, so nothing to process
		if len(self.byte_buffer) == 0: return

		# Process as many bytes / packets as possible
		while True:
			# Stop when there are no more bytes to process
			if len(self.byte_buffer) == 0: break
			# Get the packet type of the packet at the front of the buffer
			packet_type = self.byte_buffer[0]

			# BasestationLog or RobotLog. Assumption that these are the only two packets with dynamic size
			if packet_type in [BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG, BaseTypes.PACKET_TYPE_REM_ROBOT_LOG]:
				idx = 0 if packet_type == BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG else 1
				packet_size = [BaseTypes.PACKET_SIZE_REM_BASESTATION_LOG, BaseTypes.PACKET_SIZE_REM_ROBOT_LOG][idx]
				# Check if the entire packet is in the buffer
				if len(self.byte_buffer) < packet_size:	break
				# Get required bytes from buffer and process
				packet = [REM_BasestationLog, REM_RobotLog][idx]()
				packet.decode(self.byte_buffer[:packet_size])
				# Check if the entire message is in the buffer
				if len(self.byte_buffer) < packet_size + packet.messageLength: break
				# Get the message from the buffer
				message = self.byte_buffer[packet_size : packet_size + packet.messageLength]
				packet.message = message.decode()
				# Add packet to buffer
				self.packet_buffer.append(packet)
				# Remove processed bytes from buffer
				self.byte_buffer = self.byte_buffer[packet_size + packet.messageLength:]
		
			# Basestation get configuration
			else:
				packet_size = BaseTypes.PACKET_TYPE_TO_SIZE(packet_type)
				
				if len(self.byte_buffer) < packet_size: break
				# Create packet instance
				packet_obj = BaseTypes.PACKET_TYPE_TO_OBJ(packet_type)()
				# print(f"[process] Packet of size {packet_size} : {packet_type} : {type(packet_obj).__name__}")

				# Get required bytes from buffer and process
				packet_obj.decode(self.byte_buffer[:packet_size])
				# Add packet to buffer
				self.packet_buffer.append(packet_obj)
				# Remove processed bytes from buffer
				self.byte_buffer = self.byte_buffer[packet_size:]

	def hasPackets(self):
		return 0 < len(self.packet_buffer)
	
	def getNextPacket(self):
		if self.hasPackets(): return self.packet_buffer.popleft()



while True:
	try:

		latest_packets = {}
		last_robotfeedback_time = 0
		last_robotcommand_time = 0
		lastBasestationLog = ""

		# Open basestation with the basestation
		if basestation is None or not basestation.isOpen():
			basestation = utils.openContinuous(timeout=0.01)
			print("Basestation opened")
			reader = REMReader(basestation)

		# Continuously write -> read -> visualise
		while True:

			tick_required = 1./packetHz <= time.time() - lastWritten

			# ========== WRITING ========== #
			if tick_required:

				# Timing stuff
				lastWritten += 1./packetHz
				tickCounter += 1
				period = tickCounter % periodLength
				period_fraction = period / periodLength

				# Check connection with robot
				robotConnected = last_robotcommand_time - last_robotfeedback_time < 0.5

				# Update test if needed
				if doFullTest and period == 0:
					testIndex = (testIndex + 1) % len(testsAvailable)
					if testIndex == 0: testIndex = 2
					test = testsAvailable[testIndex]

				# Create and send new robot command
				cmd, cmd_log = createRobotCommand(robot_id, test, period_fraction)
				basestation.write(cmd.encode())
				last_robotcommand_time = time.time()

				# Logging
				bar = drawProgressBar(period_fraction)
				if not robotConnected:
					print(" Receiving no feedback!", end="")
				tcs, tfr = totalCommandsSent, totalFeedbackReceived
				print(f" {robot_id} - {test} {bar} {cmd_log} | {lastBasestationLog} ", end=" "*23 + "\r")



			# ========== READING ========== # 
			reader.read() # Read all available bytes
			reader.process() # Convert all read bytes into packets

			# Handle and store all new packets
			while reader.hasPackets():
				packet = reader.getNextPacket()
				latest_packets[type(packet)] = packet


			if REM_BasestationLog in latest_packets and latest_packets[REM_BasestationLog] is not None:
				lastBasestationLog = latest_packets[REM_BasestationLog].message
				latest_packets[REM_BasestationLog] = None
				if lastBasestationLog[-1] == '\n': lastBasestationLog = lastBasestationLog[:-1]



			# ========== VISUALISING ========== #

			# Break if cv2 is not imported
			if not cv2_available: break

			# Draw robot on the image
			s = 101.2
			cv2.line(img, (int(250-s/2), 250-73), (int(250+s/2), 250-73), (255,255,255),2)
			cv2.ellipse(img, (250, 250), (90, 90), -90, 35, 325, (255,255,255), 2)			

			### Draw information received from the RobotFeedback packet
			if REM_RobotFeedback in latest_packets and latest_packets[REM_RobotFeedback] is not None:
				robotFeedback = latest_packets[REM_RobotFeedback]
				latest_packets[REM_RobotFeedback] = None
				last_robotfeedback_time = time.time()

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

				dBm = -robotFeedback.rssi/2
				cv2.rectangle(img, (10, 10), (210, 20), (100, 0, 0), 2)
				cv2.rectangle(img, (10, 10), (10 + int(200*(1 - dBm/-80)), 20), (0, 255, 0), -1)

			### Draw information received from the RobotStateInfo packet
			if REM_RobotStateInfo in latest_packets and latest_packets[REM_RobotStateInfo] is not None:
				robotStateInfo = latest_packets[REM_RobotStateInfo]
				latest_packets[REM_RobotStateInfo] = None

				# XSens yaw
				px, py = rotate((250, 250), (250, 150), -robotStateInfo.xsensYaw)
				cv2.line(img, (250, 250), (int(px), int(py)), (1, 1, 1), 1)
				cv2.circle(img, (int(px), int(py)), 5, (1, 1, 1), -1)

				# Commanded yaw
				px, py = rotate((250, 250), (250, 150), -cmd.angle)
				cv2.line(img, (250, 250), (int(px), int(py)), (0, 1, 0), 1)
				cv2.circle(img, (int(px), int(py)), 5, (0, 1, 0), -1)
				
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
			

			if tick_required:
				cv2.imshow("Press esc to quit", img)
				if cv2.waitKey(1) == 27: exit()
				img *= 0.7


	except serial.SerialException as se:
		print("SerialException", se)
		basestation = None
		lastBasestationLog = ""
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
