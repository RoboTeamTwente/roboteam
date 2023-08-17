import os
import time
import serial
from bitarray import bitarray
from bitarray.util import int2ba, ba2int
from inspect import getmembers
import re

def openPort(port:str, timeout:int=None):
	""" Tries to open a serial port to the given path. Could be a basestation, an STLink programmer, or something else. The 
	settings are fixed to 115200 baud, 8N1. These are the same settings that the basestation and STLink use.

	Args:
		port (str): The port to open. E.g. "/dev/serial/by-id/usb-RTT_BaseStation-0293490234"
		timeout (int, optional): Timeout that the serial connection uses when calling e.g. readline(). Defaults to None.

	Returns:
		serial.Serial: The serial connection object. If the port could not be opened, None is returned.
	"""
	connection = None

	try:
		connection = serial.Serial(
		    port=port,
		    baudrate=115200,
		    parity=serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=timeout
		)
	except serial.serialutil.SerialException as e:
		print("[open][SerialException] Could not open port", port)

	return connection

def openContinuous(*args, **kwargs):
	""" A wrapper around openPort() that tries to open the port continuously until it succeeds. This is useful when a programmer
	or basestation keeps disconnecting and reconnecting. The function will try to open the port every 100ms until it succeeds. If
	no port is given, it will automatically search for a basestation or STLink programmer.

	Returns:
		serial.Serial: The serial connection object. If the port could not be opened, None is returned.
	"""
	connection = None
	# Counter for the spinner, which goes from ASCII 33 '!' to ASCII 126 '~'
	i = -1 

	while connection is None:
		# Increment the spinner counter. 126 - 33 = 93, so we wrap around at 93
		i = (i + 1) % 93 

		# If no port is given, try to find a basestation or STLink programmer
		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getBasestationPath()
		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getSTLinkPath()
		
		# If no port is found, print a spinner and try again
		if kwargs["port"] is None:
			print("\r[openContinuous] Basestation path not found %s " % chr(33 + i), end="")
		# If a port is found, try to open it
		else:
			connection = openPort(port=kwargs["port"], timeout=kwargs["timeout"])	
			print("\r[openContinuous] %s " % chr(33 + i), end="")
		
		time.sleep(0.1)
  
	print(f"\r[openContinuous] Basestation opened at {connection.port}")
	return connection

def getBasestationPath():
	""" Tries to find a basestation in the /dev/serial/by-id folder. If one is found, the path is returned. If not, None is returned.

	Returns:
		str: The path to the basestation, or None if no basestation is found.
	"""
	
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		basestations = [device for device in usb_devices if device.startswith("usb-RTT_BaseStation")]
		if 0 < len(basestations):
			return os.path.join("/dev/serial/by-id", basestations[0])
		return None
	except Exception:
		return None

def getSTLinkPath():
	""" Tries to find an STLink programmer in the /dev/serial/by-id folder. If one is found, the path is returned. If not, None is returned.

	Returns:
		str: The path to the STLink programmer, or None if no STLink programmer is found.
	"""
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		stlinks = [device for device in usb_devices if device.startswith("usb-STMicroelectronics_STM32_STLink")]
		if 0 < len(stlinks):
			return os.path.join("/dev/serial/by-id", stlinks[0])
		return None
	except Exception:
		return None

def printCompletePacket(rc):
	""" Takes a REM packet of any type (e.g. REM_RobotCommand or REM_Log) and prints it in a nice format. This is useful for debugging.

	Args:
		rc (REM_*): The REM packet to print.
	"""
    
	types_allowed = [int, str, bool, float]
	maxLength = max([len(k) for k, v in getmembers(rc)])
	title = re.findall(r"_(\w+) ", str(rc))[0]
	
	lines = [("┌─ %s "%title) + ("─"*100)[:maxLength*2+2-len(title)] + "┐" ]	
	members = [ [k,v] for k,v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__")]

	lines += [ "│ %s : %s │" % ( k.rjust(maxLength) , str(v).strip().ljust(maxLength) ) for k, v in members ]
	lines += [ "└" + ("─"*(maxLength*2+5)) + "┘"]
	print("\n".join(lines))

def packetToDict(rc):
	""" Takes a REM packet of any type (e.g. REM_RobotCommand or REM_Log) and converts it to a dictionary. This is useful for debugging.

	Args:
		rc (REM_*): The REM packet to convert.

	Returns:
		dict: The REM packet as a dictionary.
	"""
	types_allowed = [int, str, bool, float]
	members = { k:v for k,v in getmembers(rc) if type(v) in types_allowed and not k.startswith("__") }
	return members