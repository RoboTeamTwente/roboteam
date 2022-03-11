import sys
import math
import time
import signal
from xbox360controller import Xbox360Controller
import utils
import serial 
import numpy as np

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand as RobotCommand
from roboteam_embedded_messages.python.REM_RobotBuzzer import REM_RobotBuzzer as RobotBuzzer

robotCommand = RobotCommand()
robotBuzzer = RobotBuzzer()

last_written = time.time()
packet_Hz = 60
basestation = None

robot_id = 15
absolute_angle = 0

KICK_SPEED = 3

class JoystickWrapper:
	def __init__(self, controller):
		self.controller = controller
		self.robot_id = 0
		self.absolute_angle = 0

		self.A = False
		self.B = False
		self.X = False
		self.Y = False
		self.HAT_X = 0
		self.HAT_Y = 0
		self.command = RobotCommand()

	def get_payload(self):

		if self.controller.button_mode._value:
			print("Exiting!")
			sys.exit()

		if self.HAT_X != self.controller.hat.x:
			self.HAT_X = self.controller.hat.x
			self.robot_id += self.controller.hat.x
			print(f"Switched to ID {self.robot_id}")

		# Toggle dribbler
		self.command.dribbler = 0
		if self.controller.button_y._value and not self.Y:
			self.Y = True
			if self.command.dribbler == 0: 
				self.command.dribbler = 7
			else:
				self.command.dribbler = 0
		else:
			self.Y = False

		if self.robot_id < 0: self.robot_id = 0
		if 15 < self.robot_id : self.robot_id = 15

		self.command.doKick = False
		self.command.doChip = False

		if self.controller.button_a._value and not self.A:
			self.command.kickChipPower = KICK_SPEED
			self.command.doChip = True
			self.command.doForce = True
		self.A = self.controller.button_a._value

		if self.controller.button_b._value and not self.B:
			self.command.kickChipPower = KICK_SPEED
			self.command.doKick = True
			self.command.doForce = True
		self.B = self.controller.button_b._value


		# Angle
		if 0.3 < abs(self.controller.axis_r.x): self.absolute_angle -= self.controller.axis_r.x * 0.1
		
		# Forward backward left right
		deadzone = 0.3

		velocity_x = 0
		if deadzone < abs(self.controller.axis_l.x):
			velocity_x = ( abs(self.controller.axis_l.x) - deadzone) / (1 - deadzone)
			velocity_x *= np.sign(self.controller.axis_l.x)

		velocity_y = 0
		if deadzone < abs(self.controller.axis_l.y):
			velocity_y = ( abs(self.controller.axis_l.y) - deadzone) / (1 - deadzone)
			velocity_y *= np.sign(self.controller.axis_l.y)

		rho = math.sqrt(velocity_x * velocity_x + velocity_y * velocity_y);
		theta = math.atan2(-velocity_x, -velocity_y);

		self.command.header = BaseTypes.PACKET_TYPE_REM_ROBOT_COMMAND
		self.command.remVersion = BaseTypes.LOCAL_REM_VERSION
		self.command.id = self.robot_id

		self.command.rho = rho
		self.command.theta = theta + self.absolute_angle
		self.command.angle = self.absolute_angle


		buzzer_value = self.controller.trigger_l._value
		if 0.3 < buzzer_value:
			buzzer_command = RobotBuzzer()
			buzzer_command.header = BaseTypes.PACKET_TYPE_REM_ROBOT_BUZZER
			buzzer_command.remVersion = BaseTypes.LOCAL_REM_VERSION
			buzzer_command.id = self.robot_id
			buzzer_command.period = int(buzzer_value * 1000)
			buzzer_command.duration = 0.1

			message = self.command.encode()
			#message = np.concatenate( (self.command.encode(), buzzer_command.encode()) )
			print(message)
			return message

		return self.command.encode()


print(Xbox360Controller.get_available())

# wrappers = [JoystickWrapper(Xbox360Controller(i)) for i in range(len(Xbox360Controller.get_available()))]
wrappers = [JoystickWrapper(controller) for controller in Xbox360Controller.get_available()]

while True:
	# Open basestation with the basestation
	if basestation is None or not basestation.isOpen():
		basestation = utils.openContinuous(timeout=0.001)

	try:
		while True:
			# Run at {packet_Hz}fps
			if 1./packet_Hz <= time.time() - last_written:
				# Timing stuff
				last_written += 1./packet_Hz
				
				for wrapper in wrappers:
					basestation.write(wrapper.get_payload())

				packet_type = basestation.read(1)
				if len(packet_type) == 0:
					continue

				packetType = packet_type[0]
				
				if packetType == BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG:
					logmessage = basestation.readline().decode()
					print(logmessage)

			time.sleep(0.005)
		signal.pause()

	except serial.SerialException as se:
		print("SerialException", se)
		basestation = None
	except serial.SerialTimeoutException as ste:
		print("SerialTimeoutException", ste)
	except KeyError:
		print("[Error] KeyError", e, "{0:b}".format(int(str(e))))
	except Exception as e:
		print("[Error]", e)

print("Exiting")