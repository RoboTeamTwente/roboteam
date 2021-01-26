import time
import math
from inspect import getmembers
import utils
import serial
import numpy as np

try:
	from rem import rem
except ImportError:
	print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
	print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
	print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
	exit()

def printRobotCommand(rc):
	print(">>>>>>>> RobotCommand >>>>>>>>");
	print("            id : %d" % rc.id);
	print("        doKick : %d" % rc.doKick);
	print("        doChip : %d" % rc.doChip);
	print("       doForce : %d" % rc.doForce);
	print("useCameraAngle : %d" % rc.useCameraAngle);
	print("           rho : %.4f" % rc.rho);
	print("         theta : %.4f" % rc.theta);
	print("         angle : %.4f" % rc.angle);
	print("   cameraAngle : %.4f" % rc.cameraAngle);
	print("      dribbler : %d" % rc.dribbler);
	print(" kickChipPower : %d" % rc.kickChipPower);
	print("angularControl : %d" % rc.angularControl);
	print("      feedback : %d" % rc.feedback);

def rand(): 
	return 0.5 < np.random.rand()

written = 0

lastWritten = time.time()
lastHz = time.time()

ser = None

id_ = 0
sentCounter = 0
rho = 0
theta = 0
angle = 0
cameraAngle = 0

feedbackTracker = [0] * 16
print(feedbackTracker)

while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous(timeout=0.01)

	try:
		# Continuously read and print messages from the basestation
		while True:
			# if(1000/60. < time.time() - lastWritten):
			# 	sentCounter += 1
			# 	if sentCounter % 60 == 0:
			# 		print("Hz : %.2f" % (60 / (time.time()-timerHz)))
			# 		ser.write(bytes([utils.PACKET_TYPE["BASESTATION_GET_STATISTICS"]]))
			# 		timerHz = time.time()
			# 	else:						
			# 		p.setID(sentCounter % 5)
			# 		ser.write(p.getBytes())
			# 	lastWritten = time.time()

			if 1./60 < time.time() - lastWritten:
				# print("Tick")
				cmd = rem.ffi.new("RobotCommand*")
				cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
				cmd.id = 3

				cmd.rho = 0
				cmd.theta = 0
				cmd.angle = np.sin(sentCounter / 20.) * math.pi / 2
				# print(cmd.angle)
				cmd.cameraAngle = 0

				# hijack packet header to ask for statistics. Big hack and drops a command packet. Bad, bad, bad.
				if  1. < time.time() - lastHz:
					string = ""
					for i in range(len(feedbackTracker)):
						string += "%d = %d | " % (i, feedbackTracker[i])
					print("Feedback |", string)
					feedbackTracker = [0] * len(feedbackTracker)
					lastHz = time.time()			

				payload = rem.ffi.new("RobotCommandPayload*")
				rem.lib.encodeRobotCommand(payload, cmd) 

				# printRobotCommand(cmd)
				# for iP in range(rem.lib.PACKET_SIZE_ROBOT_COMMAND):
				# 	print("%d\t"%iP, "{0:b}".format(payload.payload[iP]).zfill(8), "\t %d" % payload.payload[iP], "\t", hex(payload.payload[iP]))
				
		
				ser.write(payload.payload)



				rho = (rho + 0.01) % 3
				theta = (theta + 0.2) % 3
				# angle = (angle - 0.05)
				


				if 2 * math.pi < angle:
					angle = 0
				if angle < 0:
					angle = math.pi * 2

				cameraAngle = (cameraAngle + 0.4) % 3


				sentCounter += 1
				id_ = (id_ + 1) % 16
				lastWritten = time.time()



			packet_type = ser.read(1)
			if len(packet_type) == 0:
				continue

			packetType = packet_type[0]


			if packetType == rem.lib.PACKET_TYPE_BASESTATION_LOG:
				print("[LOG]" + ser.readline().decode(), end="")
				continue


			if packetType == rem.lib.PACKET_TYPE_ROBOT_COMMAND:
				print("RobotCommand received")
				packet = packet_type + ser.read(rem.lib.PACKET_SIZE_ROBOT_COMMAND - 1)
				payload = rem.ffi.new("RobotCommandPayload*")
				payload.payload = packet



				cmd = rem.ffi.new("RobotCommand*")
				rem.lib.decodeRobotCommand(cmd, payload)
				
				printRobotCommand(cmd)

			if packetType == rem.lib.PACKET_TYPE_ROBOT_FEEDBACK:
				# print("RobotFeedback received")
				packet = packet_type + ser.read(rem.lib.PACKET_SIZE_ROBOT_FEEDBACK - 1)
				payload = rem.ffi.new("RobotFeedbackPayload*")
				payload.payload = packet

				cmd = rem.ffi.new("RobotFeedback*")
				rem.lib.decodeRobotFeedback(cmd, payload)
				# for k, v in getmembers(cmd):
				# 	print(k.rjust(20), v)
				# print("\n")

				feedbackTracker[cmd.id] += 1


			if packetType == rem.lib.PACKET_TYPE_BASESTATION_STATISTICS:
				print("Basestation statistics received")
				packet = ser.read(rem.lib.PACKET_TYPE_BASESTATION_STATISTICS - 1)
				string = ""
				for i in range(16):
					string += " | %d %d %d" % (i, packet[i*2], packet[i*2+1])
				print(string)
				print("\n")

			time.sleep(0.01)

	except serial.SerialException as se:
		print("se", se)
		ser = None
	except serial.SerialTimeoutException as ste:
		print("ste", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		# ser = None