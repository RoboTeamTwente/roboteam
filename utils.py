import time
import serial
from bitarray import bitarray
from bitarray.util import int2ba, ba2int

def openPort(port="/dev/ttyACM1", suppressError = False, timeout=None):
	ser = None
	try:
		ser = serial.Serial(
		    port=port,
		    baudrate=115200,
		    parity=serial.PARITY_ODD,
		    stopbits=serial.STOPBITS_TWO,
		    bytesize=serial.SEVENBITS,
		    timeout=timeout
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

class Feedback:
	def __init__(self, byts):
		self.bytes = byts
		self.decode()

	def decode(self):
		self.id = ba2int(self.bytes[0:8])
		self.xSensCalibrated = self.bytes[8] == 1
		self.batteryLow = self.bytes[9] == 1
		self.ballSensorWorking = self.bytes[10] == 1
		self.hasBall = self.bytes[11] == 1
		self.ballPosition = ba2int(self.bytes[12:16])
		self.genevaWorking = self.bytes[16] == 1
		self.genevaState = ba2int(self.bytes[17:24])
		self.rho = ba2int(self.bytes[24:35])
		self.angle = ba2int(self.bytes[35:45])
		self.theta = ba2int(self.bytes[45:56])
		self.hasLockedWheel = self.bytes[56] == 1
		self.signalStrength = ba2int(self.bytes[57:64])


	def __repr__(self):
		string = "Feedback(ID=%d, xsens=%d, bat=%d, bs=%d, hb=%d, bp=%d, gw=%d, geneva=%d, rho=%d, angle=%d, theta=%d, lw=%d, ssid=%d)" 
		string = string % (
			self.id,
			self.xSensCalibrated,
			self.batteryLow,
			self.ballSensorWorking,
			self.hasBall,
			self.ballPosition,
			self.genevaWorking,
			self.genevaState,
			self.rho,
			self.angle,
			self.theta,
			self.hasLockedWheel,
			self.signalStrength
		)
		return string