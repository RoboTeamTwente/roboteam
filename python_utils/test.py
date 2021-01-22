import time
import serial

try:
	from rem import rem
except ImportError:
	print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
	print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
	print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
	exit()

port = "/dev/serial/by-id/usb-STMicroelectronics_STM32_STLink_066CFF575251717867231650-if02"

ser = serial.Serial(
	port=port,
	baudrate=115200,
	timeout=0.01
)

lastWritten = time.time()

n = 0
theta = 0.
rho = 0.

while True:

	if 1 < time.time() - lastWritten:
		print("-")
		rho += 0.1
		theta += 0.14
		theta = 0

		if 2. < rho: rho = 0
		if 2. < theta: theta = 0

		cmd = rem.ffi.new("RobotCommand*")
		cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
		cmd.id = n
		
		cmd.rho = rho
		cmd.theta = theta

		payload = rem.ffi.new("RobotCommandPayload*")
		rem.lib.encodeRobotCommand(payload, cmd) 

		print("[python] Sending packet with id=%d rho=%.2f theta=%.2f" % (cmd.id, cmd.rho, cmd.theta))
		ser.write(payload.payload)

		n = (n + 1) % 16
		lastWritten = time.time()
	
	line = ser.readline().decode()
	if 0 < len(line):
		print(line, end="")
	time.sleep(0.01)
