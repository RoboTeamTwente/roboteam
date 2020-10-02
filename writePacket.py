import time
import serial
import math
from bitarray import bitarray

import utils

p = utils.Packet()

p.setID(5)

angle = 0
counter = 0

written = 0

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous()

	try:
		# Continuously read and print messages from the basestation
		while True:
			ser.write(p.array.tobytes())
			written += 1

			response = ser.readline().decode("utf-8")
			cFeedback = int(response.split(" ")[0])
			# print(cFeedback, response)
			print(written, cFeedback, cFeedback / written)
			
			# Angle
			angle += 0
			angle %= 2**10
			p.setAngularVelocity(angle)

			# Velocity
			counter += 1
			vel = math.sin(counter/10)*100

			if vel < 0:
				p.setTheta(int(2**10)-1)
			else:
				p.setTheta(0)

			p.setRho(int(abs(vel)))

			time.sleep(0.01)

	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		ser = None