import time
import serial

def open(port="/dev/ttyACM1", suppressError = False):
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
		ser = open(**kwargs, suppressError = True)	
		
		i = (i + 1) % 93
		print("\r[openContinuous] %s " % chr(33 + i), end="")
		
		time.sleep(0.1)
	print("\r[openContinuous] Basestation opened")
	return ser

ser = None
while True:
	# Open connection with the basestation
	if ser is None or not ser.isOpen():
		ser = openContinuous()

	try:
		# Continuously read and print messages from the basestation
		while True:
			print(ser.readline().decode("utf-8"), end="")
	except Exception as e:
		print("[Error]", e)
		# Reset the connection to the basestation
		ser = None