import time
import serial
import math
from bitarray import bitarray

import utils

p = utils.Packet()

p.setID(5)

angle = 0
counter = 0

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous()

	try:
		# Continuously read and print messages from the basestation
		while True:
			ser.write(p.array.tobytes())
			# while ser.available():
			print(ser.readline().decode("utf-8"), end="")
			
			# Angle
			angle += 0
			angle %= 2**10
			p.setAngularVelocity(angle)

			# Velocity
			counter += 1
			vel = math.sin(counter/10)*300

			if vel < 0:
				p.setTheta(int(2**10)-1)
			else:
				p.setTheta(0)

			p.setRho(int(abs(vel)))

			time.sleep(0.016)

	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		ser = None