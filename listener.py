import time
import serial
import utils

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = utils.openContinuous()

	try:
		# Continuously read and print messages from the basestation
		while True:
			print(ser.readline().decode("utf-8"), end="")
	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		ser = None