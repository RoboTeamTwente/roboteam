import os
import time
import serial
from bitarray import bitarray
from bitarray.util import int2ba, ba2int

def openPort(port=None, suppressError = True, timeout=None):
	ser = None
	if port is None:
		return None

	try:
		ser = serial.Serial(
		    port=port,
		    baudrate=115200,
		    parity=serial.PARITY_NONE,
		    stopbits=serial.STOPBITS_ONE,
		    bytesize=serial.EIGHTBITS,
		    timeout=timeout
		)
	except serial.serialutil.SerialException as e:
		print("[open]", e)
		# if not suppressError:

	return ser

def openContinuous(*args, **kwargs):
	ser = None
	i = -1

	while ser is None:
		i = (i + 1) % 93

		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getBasestationPath()
		
		if "port" not in kwargs or kwargs["port"] is None:
			kwargs["port"] = getSTLinkPath()
		
		if kwargs["port"] is None:
			print("\r[openContinuous] Basestation path not found %s " % chr(33 + i), end="")
		else:
			ser = openPort(**kwargs, suppressError = True)	
			print("\r[openContinuous] %s " % chr(33 + i), end="")
		
		time.sleep(0.1)
	print(f"\r[openContinuous] Basestation opened at {ser.port}")
	return ser

def getBasestationPath():
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		basestations = [device for device in usb_devices if device.startswith("usb-RTT_BaseStation")]
		if 0 < len(basestations):
			return os.path.join("/dev/serial/by-id", basestations[0])
		return None
	except Exception:
		return None

def getSTLinkPath():
	try:
		usb_devices = os.listdir("/dev/serial/by-id")
		stlinks = [device for device in usb_devices if device.startswith("usb-STMicroelectronics_STM32_STLink")]
		if 0 < len(stlinks):
			return os.path.join("/dev/serial/by-id", stlinks[0])
		return None
	except Exception:
		return None
