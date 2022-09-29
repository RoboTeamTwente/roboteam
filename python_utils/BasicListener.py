import utils
import serial

print("Opening basestation..")
basestation = utils.openContinuous(timeout=0.01)
print("Opened", basestation.port)

while True:
	print(basestation.readline().decode())