import utils
import serial

print("Opening basestation..")
basestation = utils.openContinuous(timeout=0.01)
print("Opened", basestation.port)

# while True:
# 	line = basestation.readline().decode()
# 	if 0 < len(line): print(line)