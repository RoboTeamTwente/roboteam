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

class RobotCommand:

	array = bitarray(10*8)

	def __init__(self):
		self.reset()
		self.setPacketType()

	def getBytes(self):
		return self.array.tobytes()

	def reset(self):
		self.array[:] = 0

	def setPacketType(self):
		self.array[0:8] = int2ba(PACKET_TYPE["ROBOT_COMMAND"], length=8)
	#---
	def setID(self, robot_id):
		self.array[8:16] = int2ba(robot_id, length=8)
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
		self.type = ba2int(self.bytes[0:8])
		self.id = ba2int(self.bytes[8:16])
		self.xSensCalibrated = self.bytes[16] == 1
		self.batteryLow = self.bytes[17] == 1
		self.ballSensorWorking = self.bytes[18] == 1
		self.hasBall = self.bytes[19] == 1
		self.ballPosition = ba2int(self.bytes[20:24])
		self.genevaWorking = self.bytes[24] == 1
		self.genevaState = ba2int(self.bytes[25:32])
		self.rho = ba2int(self.bytes[32:43])
		self.angle = ba2int(self.bytes[43:53])
		self.theta = ba2int(self.bytes[53:64])
		self.hasLockedWheel = self.bytes[64] == 1
		self.signalStrength = ba2int(self.bytes[65:72])


	def __repr__(self):
		string = "Feedback(%s, ID=%d, xsens=%d, bat=%d, bs=%d, hb=%d, bp=%d, gw=%d, geneva=%d, rho=%d, angle=%d, theta=%d, lw=%d, rssi=-%d)" 
		string = string % (
			getPacketType(self.type),
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

PACKET_TYPE  = {
	"ROBOT_COMMAND"                  : 0b01100110,
	"ROBOT_FEEDBACK"                 : 0b00001111,
	"ROBOT_SET_SETTINGS"             : 0b00110011,
	"ROBOT_GET_SETTINGS"             : 0b00111100,
	"ROBOT_SETTINGS"                 : 0b01010101,
	"ROBOT_ALERT"                    : 0b01011010,

	"BASESTATION_SET_SETTINGS"       : 0b10010110,
	"BASESTATION_GET_SETTINGS"       : 0b10011001,
	"BASESTATION_SETTINGS"           : 0b10100101,
	"BASESTATION_GET_STATISTICS"     : 0b10101010,
	"BASESTATION_STATISTICS"         : 0b11000011,
	"BASESTATION_ALERT"              : 0b11001100,

	"RESERVED_1"                     : 0b01101001,
	"RESERVED_2"                     : 0b11110000
}

PACKET_SIZE = {
	PACKET_TYPE["ROBOT_COMMAND"] 			: 10,
	PACKET_TYPE["ROBOT_FEEDBACK"] 			: 9,
	PACKET_TYPE["ROBOT_SET_SETTINGS"] 		: 0,
	PACKET_TYPE["ROBOT_GET_SETTINGS"] 		: 0,
	PACKET_TYPE["ROBOT_SETTINGS"] 			: 0,
	PACKET_TYPE["ROBOT_ALERT"] 				: 0,

	PACKET_TYPE["BASESTATION_SET_SETTINGS"] : 0,
	PACKET_TYPE["BASESTATION_GET_SETTINGS"] : 0,
	PACKET_TYPE["BASESTATION_SETTINGS"] 	: 0,
	PACKET_TYPE["BASESTATION_GET_STATISTICS"] : 0 ,
	PACKET_TYPE["BASESTATION_STATISTICS"] 	: 33,
	PACKET_TYPE["BASESTATION_ALERT"] 		: 0,

	PACKET_TYPE["RESERVED_1"] 				: 0,
	PACKET_TYPE["RESERVED_2"] 				: 0
}

def getPacketType(val):
	for k in PACKET_TYPE.keys():
		if PACKET_TYPE[k] == val:
			return k
	return "UNKNOWN_TYPE"