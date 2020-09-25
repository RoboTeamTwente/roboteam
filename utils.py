import time
import serial
from bitarray import bitarray
from bitarray.util import int2ba

def openPort(port="/dev/ttyACM1", suppressError = False):
	ser = None
	try:
		ser = serial.Serial(
		    port=port,
		    baudrate=115200,
		    parity=serial.PARITY_ODD,
		    stopbits=serial.STOPBITS_TWO,
		    bytesize=serial.SEVENBITS
		)
	except serial.serialutil.SerialException as e:
		if not suppressError:
			print("[open]", e)

	return ser

def openContinuous(*args, **kwargs):
	ser = None
	i = -1
	while ser is None:
		ser = openPort(**kwargs, suppressError = True)	
		
		i = (i + 1) % 93
		print("\r[openContinuous] %s " % chr(33 + i), end="")
		
		time.sleep(0.1)
	print("\r[openContinuous] Basestation opened")
	return ser

class Packet:

	array = bitarray(10*8)

	def __init__(self):
		print("New packet")
		self.reset()

	def reset(self):
		self.array[:] = 0

	def setID(self, robot_id):
		self.array[0:8] = int2ba(robot_id, length=8)
	#---
	def setEmpty(self):
		self.array[8:16] = 0
	#---
	def setRho(self, rho):
		self.array[16:27] = int2ba(rho, length=11)
	#---
	def setTheta(self, theta):
		self.array[27:38] = int2ba(theta, length=11)
	#---
	def setAngularVelocity(self, velocity):
		self.array[38:48] = int2ba(velocity, length=10)
	#---#---
	def setPower(self, power):
		self.array[48:56] = int2ba(power, length=8)
	#---
	def doKick(self, kick):
		self.array[56] = kick
	
	def doChip(self, chip):
		self.array[57] = chip
	
	def setForced(self, forced):
		self.array[58] = forced

	def setDebugInfo(self, debugInfo):
		self.array[59] = debugInfo

	def useCamInfo(self, useInfo):
		self.array[60] = useInfo

	def setGenevaDrive(self, geneva):
		self.array[61:64] = int2ba(geneva, length=3)
	#---
	def setDribblerVelocity(self, velocity):
		self.array[64:69] = int2ba(velocity, length=5)

	def setCameraRotation(self, rotation):
		self.array[69:80] = int2ba(rotation, length=11)
	#---#---

	def __repr__(self):
		return "[Packet]" + str(self.array)