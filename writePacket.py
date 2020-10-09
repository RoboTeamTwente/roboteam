import time
import math
import bitarray
from bitarray import bitarray as ba
from bitarray.util import int2ba

import utils

p = utils.RobotCommand()

p.setID(15)

angle = 0
counter = 0

written = 0

lastWritten = time.time()

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous(timeout=0.1)

	try:
		# Continuously read and print messages from the basestation
		while True:

			if(0.1 < time.time() - lastWritten):
				# print("Written")
				ser.write(p.getBytes())
				lastWritten = time.time()

			response = ser.read(9)
			if 0 < len(response):
				# print(response)
				x = ba()		
				angle = 0
				# try:
				x.frombytes(response)
				u = utils.Feedback(x)
				# except Exception as e:
				# 	pass
				# print(u)
				print("Xsens=%d"%u.xSensCalibrated, "Angle=%d"%u.angle, len(response), response)
				print(utils.Feedback(x))
				

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
		raise e