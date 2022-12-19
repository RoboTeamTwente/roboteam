"""
This file fakes a basestation / robot serial connection. It creates a pty (pseudo-terminal), 
that can act as a serial device. This pty can be opened with the Pyserial library, exactly the same as a 
basestation / programmer is opened. Useful to test scripts without actually having a basestation or programmer.
Doesn't support Windows.

Fake serial device : https://stackoverflow.com/questions/2291772/virtual-serial-device-in-python
"""

import os
import pty
import threading
import time
import serial
import numpy as np

import roboteam_embedded_messages.python.REM_BaseTypes as REM_BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_Log import REM_Log

def getValue(a, b, p):
	return (b-a) * p + a



class SerialSimulator:
	def __init__(self):
		self.master, self.slave = pty.openpty()
		self.serial_name = os.ttyname(self.slave)
		self.running = False

	def getSerialName(self):
		return self.serial_name

	def run(self):
		self.thread = threading.Thread(target=self.fakeREM_RobotCommand)
		self.running = True
		self.thread.start()

	def stop(self):
		print("[SerialSimulator][stop] Stopping and joining thread..")
		self.running = False
		self.thread.join()

	def fakeREM_RobotCommand(self):
		time.sleep(0.5)
		start_time = time.time()
		while self.running:
			current_time = time.time() - start_time

			# Default stuff
			cmd = REM_RobotFeedback()
			cmd.header = REM_BaseTypes.REM_PACKET_TYPE_REM_ROBOT_FEEDBACK
			cmd.remVersion = REM_BaseTypes.REM_LOCAL_VERSION
			cmd.payloadSize = REM_BaseTypes.REM_PACKET_SIZE_REM_ROBOT_FEEDBACK
			cmd.fromRobot = 1
			cmd.toPC = True
			# The actual feedback
			cmd.rho = getValue(REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_RHO_MIN, REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_RHO_MAX, current_time%1)
			cmd.theta = getValue(REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_THETA_MIN, REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_THETA_MAX, 0.2*current_time%1)
			cmd.angle = getValue(REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_ANGLE_MIN, REM_BaseTypes.REM_PACKET_RANGE_REM_ROBOT_FEEDBACK_ANGLE_MAX, current_time%1)

			os.write(self.master, cmd.encode())
			time.sleep(0.02) # 50Hz

			if np.random.rand() < 0.02:
				from_bot = 0.25 < np.random.rand()
				message = ["Basestation log!\n", "Robot log!\n"][from_bot].encode()
				rem_log = REM_Log()
				rem_log.header = REM_BaseTypes.REM_PACKET_TYPE_REM_LOG
				rem_log.toPC = True
				rem_log.fromRobotId = from_bot * np.random.randint(0, 16)
				rem_log.fromBS = not from_bot
				rem_log.remVersion = REM_BaseTypes.REM_LOCAL_VERSION
				rem_log.payloadSize = REM_BaseTypes.REM_PACKET_SIZE_REM_LOG + len(message)

				os.write(self.master, rem_log.encode().tobytes() + message)

if __name__ == "__main__":
	print("SerialSimulator.py")

	ss = SerialSimulator()
	ss.run()

	import REMParser
	parser = REMParser.REMParser(device=serial.Serial(ss.getSerialName()))

	# Atm, this doesn't seem to work. No packets arrive
	try:
		while(True):
			time.sleep(0.01)
			parser.read()
			parser.process()
			while parser.hasPackets():
				packet = parser.getNextPacket()
				print("\n\nReceived", type(packet).__name__)
	except KeyboardInterrupt as ki:
		ss.stop()

