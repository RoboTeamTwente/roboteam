import matplotlib.pyplot as plt
import os
files = [file for file in os.listdir() if file.startswith("robotStateInfo") and file.endswith(".csv")]


# lines = open("robotStateInfo_1632500763.csv", "r").read().strip().split("\n")
# lines = open("robotStateInfo_1632501235.csv", "r").read().strip().split("\n")
lines = open("robotStateInfo_1632501996.csv", "r").read().strip().split("\n")


timestamps = []
yaws = []
kPs, kIs, kDs, Is = [], [], [], []

for line in lines[100:500]:
	ts, yaw, kP, kI, kD, I = line.split(" ")
	ts = int(float(ts) * 1000)
	yaw = float(yaw)
	timestamps.append(ts)
	yaws.append(yaw)

	kPs.append(float(kP))
	kIs.append(float(kI))
	kDs.append(float(kD))
	Is.append(float(I)*20)


plt.plot(timestamps, yaws)
plt.plot(timestamps, Is)
plt.show()

