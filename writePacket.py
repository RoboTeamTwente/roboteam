import time
import math
import bitarray
from bitarray import bitarray as ba
from bitarray.util import int2ba

import utils

p = utils.RobotCommand()

p.setID(14)

angle = 0
counter = 0
sentCounter = 0

written = 0

lastWritten = time.time()

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous(timeout=0.01)

	try:
		# Continuously read and print messages from the basestation
		while True:

			if(0.1 < time.time() - lastWritten):
				sentCounter += 1
				if sentCounter % 50 == 0:	ser.write(bytes([utils.PACKET_TYPE["BASESTATION_GET_STATISTICS"]]))
				else:						ser.write(p.getBytes())
				lastWritten = time.time()

			packet_type = ser.read(1)
			if len(packet_type) == 0:
				continue

			# Read packet size
			packet_size = utils.PACKET_SIZE[packet_type[0]]
			# Read packet
			response = None
			if packet_type[0] == utils.PACKET_TYPE["BASESTATION_LOG"]:
				response = ser.readline()
			else:
				response = ser.read(packet_size-1)

			# Decode packet
			
			if 0 < len(response) and packet_type[0] == utils.PACKET_TYPE["ROBOT_FEEDBACK"]:
				feedback = ba()		
				feedback.frombytes(packet_type + response)
				u = utils.Feedback(feedback)
				# print(utils.Feedback(feedback))
				continue
				
			if 0 < len(response) and packet_type[0] == utils.PACKET_TYPE["BASESTATION_STATISTICS"]:
				string = ""
				for i in range(16):
					string += " | %d %d %d" % (i, response[i*2], response[i*2+1])
				print(string)
				continue

			if 0 < len(response) and packet_type[0] == utils.PACKET_TYPE["BASESTATION_LOG"]:
				print("[LOG]" + response.decode(), end="")
				continue

			if 0 < len(response):
				print("Received something..", packet_type[0], packet_size)


			# ser.write(p.array.tobytes())
			# written += 1

			# response = ser.readline().decode("utf-8")
			# cFeedback = int(response.split(" ")[0])
			# print(written, cFeedback, cFeedback / written)
			
			# # Angle
			# angle += 0
			# angle %= 2**10
			# p.setAngularVelocity(angle)

			# # Velocity
			# counter += 1
			# vel = math.sin(counter/10)*100

			# if vel < 0:
			# 	p.setTheta(int(2**10)-1)
			# else:
			# 	p.setTheta(0)

			# p.setRho(int(abs(vel)))

			# time.sleep(0.05)

	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		ser = None