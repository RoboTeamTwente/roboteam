import time
import serial
import numpy as np
from inspect import getmembers

try:
	from rem import rem
except ImportError:
	print("[Error] Could not import rem, the roboteam_embedded_messages python bindings")
	print("[Error] Generate the bindings by going to ./roboteam_embedded_messages/python_bindings, and execute:")
	print("[Error] $ python generate.py --includes ../include/*  --name rem --output ../../rem")
	exit()

def printRobotCommand(rc):
	print(">>>>>>>> RobotCommand >>>>>>>>");
	print("            id : %d" % rc.id);
	print("        doKick : %d" % rc.doKick);
	print("        doChip : %d" % rc.doChip);
	print("       doForce : %d" % rc.doForce);
	print("useCameraAngle : %d" % rc.useCameraAngle);
	print("           rho : %.4f" % rc.rho);
	print("         theta : %.4f" % rc.theta);
	print("         angle : %.4f" % rc.angle);
	print("   cameraAngle : %.4f" % rc.cameraAngle);
	print("      dribbler : %d" % rc.dribbler);
	print(" kickChipPower : %d" % rc.kickChipPower);
	print("angularControl : %d" % rc.angularControl);
	print("      feedback : %d" % rc.feedback);

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
interval = 2

def rand(): 
	return 0.5 < np.random.rand()

while True:

	if interval < time.time() - lastWritten:
		print("-")
		rho += 0.1
		theta += 0.14

		if 2. < rho: rho = 0
		if 2. < theta: theta = 0

		cmd = rem.ffi.new("RobotCommand*")
		cmd.header = rem.lib.PACKET_TYPE_ROBOT_COMMAND
		cmd.id = n
		print("n=%d id=%d" % (n, cmd.id))
		
		cmd.doKick = rand()
		cmd.doChip = rand()
		cmd.doForce = rand()
		cmd.dribbler = 1
		cmd.kickChipPower = 2



		cmd.rho = rho
		cmd.theta = theta

		printRobotCommand(cmd)
		print("\n")

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
