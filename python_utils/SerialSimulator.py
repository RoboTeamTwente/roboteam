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

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback


class SerialSimulator:
	def __init__(self):
		self.master, self.slave = pty.openpty()
		self.serial_name = os.ttyname(self.slave)
	def getSerialName(self):
		return self.serial_name

	def run(self):
		self.thread = threading.Thread(target=self.fakeREM_RobotCommand)
		self.thread.start()

	def fakeREM_RobotCommand(self):
		while True:
			time.sleep(1)
			print("Running!")
			cmd = REM_RobotCommand()
			cmd.header = REM_BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
			cmd.id = 0
			os.write(self.master, cmd.encode())

if __name__ == "__main__":
	print("SerialSimulator.py")
	ss = SerialSimulator()
	ss.run()

	import REMParser
	parser = REMParser.REMParser(device=serial.Serial(ss.getSerialName()))
	print(parser)

	while(True):
		time.sleep(1)
		parser.read()
		parser.process()