import matplotlib.pyplot as plt
import os
import numpy as np
import sys
import re
import heapq
from matplotlib.offsetbox import AnchoredText

Statetimestamps = []
XsensX, XsensY, XsensYaw = [], [], [] #estimated x-, y accelerations and absolute angle
rateOfTurn = [] #estimated angular velocity
W1, W2, W3, W4 = [], [], [], [] #estimated wheel speeds
DribbleSpeed, FilteredDribbleSpeed, DribbleSpeedBeforeGotBall = [], [], [] #dribbler speeds
XInt, YInt, WInt, YawInt, W1Int, W2Int, W3Int, W4Int = [], [], [], [], [], [], [], [] #integral value from the I part of the PIDs

Feedbacktimestamps = []
BatteryLvl = [] 
XsensCalibrated = [] # true if Xsens is calibrated
BallSensorWorking, BallSensorSeesBall, DribblerSeesBall = [], [], [] #ballsensor stuff
CapacitorCharged = [] # indicates if the capacitor is charged for kicking/chipping
BallPos = [] # ball sensor position
RhoE, ThetaE = [], [] #estimated movement [m/s] and direction of movement [rad]
VexLocal, VeyLocal = [], [] #estimated local x- and y velocities calculated from gamma and phi
VexGlobal, VeyGlobal = [], [] #estimated global x- and y velocities calculated from estimated angle
wheelLocked, wheelBraking = [], [] #true if wheel is locked or braking (one bit per wheel)

Commandedtimestamps = []
DoKick, DoChip = [], [] # true if robot should kick or chip
KickAtAngle = [] #true if robot should kick at a certain angle (for orbit skill)
DoForce = [] # true if robot should kick even if ballsensor doesn't say it has the ball
UseCameraAngle = [] # true if robot should use the info in camera angle
Rho, Theta = [], [] #commanded movement [m/s] and direction of movement [rad]
Angle, AngleVel = [], [] #commanded angle and angular velocity
Vx, Vy = [], [] #commanded x- and y velocities calculated from rho and theta
Wc1, Wc2, Wc3, Wc4 = [], [], [], [] #commanded wheel speeds calculated from Vx and Vy

plotsAvailable = ["x", "y", "w", "yaw", "wheels", "integral", "integral-wheels", "rho", "theta", "ballsensor", "dribbler"]
# Parse input arguments 
try:
	if len(sys.argv) not in [2,3]:
		raise Exception("Error : Invalid number of arguments. Expected PID-index")
	plotID = sys.argv[1]
	if plotID not in plotsAvailable:
		raise Exception("Error : Unknown PID index %s. Choose one of the following : %s" % (plotID, ", ".join(plotsAvailable)))
except Exception as e:
	print(e)
	print("Error : Run script with \"python plotPID.py PID-index\"")
	exit()

lastfile = 0
if len(sys.argv) == 3: lastfile = int(sys.argv[2])
files = [file for file in os.listdir("logs") if file.startswith("robotStateInfo") and file.endswith(".csv")]
filenr = heapq.nlargest(lastfile+1, files)
filenr = re.findall('[0-9]+', str(filenr)) #find the time of the newest robotStateInfo file
StateInfoLines = open("logs/robotStateInfo_" + filenr[lastfile] + ".csv", "r").read().strip().split("\n")
CommandedLines = open("logs/robotCommand_" + filenr[lastfile] +  ".csv", "r").read().strip().split("\n")
robotFeedbackLines = open("logs/robotFeedback_" + filenr[lastfile] + ".csv", "r").read().strip().split("\n")

for line in StateInfoLines:
	ts, xsensx, xsensy, xsensyaw, rot, w1, w2, w3, w4, dribblerSpeed, filteredDribbleSpeed, dribbleSpeedBeforeGotBall, xint, yint, wint, yawint, w1int, w2int, w3int, w4int = line.split(" ")[:20]
	ts = int(float(ts) * 1000)/1000
	Statetimestamps.append(ts)
	
	XsensX.append(float(xsensx))
	XsensY.append(float(xsensy))
	XsensYaw.append(float(xsensyaw))
	rateOfTurn.append(float(rot))
	W1.append(float(w1))
	W2.append(float(w2))
	W3.append(float(w3))
	W4.append(float(w4))
	DribbleSpeed.append(float(dribblerSpeed))
	FilteredDribbleSpeed.append(float(filteredDribbleSpeed))
	DribbleSpeedBeforeGotBall.append(float(dribbleSpeedBeforeGotBall))
	XInt.append(float(xint))
	YInt.append(float(yint))
	WInt.append(float(wint))
	YawInt.append(float(yawint))
	W1Int.append(float(w1int))
	W2Int.append(float(w2int))
	W3Int.append(float(w3int))
	W4Int.append(float(w4int))
	
	
for line in CommandedLines: 
	ts, doKick, doChip, kickAtAngle, doForce, useCameraAngle, rho, theta, ang, angvel, cameraAngle, dribbler, kickChipPower, useAbsoluteAngle = line.split(" ")[:14]
	ts = int(float(ts) * 1000)/1000
	Commandedtimestamps.append(ts)
	
	Angle.append(float(ang))
	AngleVel.append(float(angvel))
	Rho.append(float(rho))
	Theta.append(float(theta))
	curVx = float(rho)*np.cos(float(theta))
	curVy = float(rho)*np.sin(float(theta))
	Vx.append(curVx)
	Vy.append(curVy)
	Wc1.append((curVx * np.cos(30 * np.pi/180) + curVy * np.sin(30 * np.pi/180))/0.028  + float(angvel) * 0.081 / 0.028) #equations from body2wheels in stateControl.c
	Wc2.append((curVx * np.cos(60 * np.pi/180) + curVy * -np.sin(60 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028) 
	Wc3.append((curVx * -np.cos(60 * np.pi/180) + curVy * -np.sin(60 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028)  
	Wc4.append((curVx * -np.cos(30 * np.pi/180) + curVy * np.sin(30 * np.pi/180))/0.028 + float(angvel) * 0.081 / 0.028) 

for line in robotFeedbackLines:
	ts, batteryLevel, XsensCalibrated, ballSensorWorking, ballSensorSeesBall, dribblerSeesBall, capacitorCharged, ballPos, rhoE, thetaE, angle, wheelLocked, wheelBraking, rssi = line.split(" ")[:14]
	ts = int(float(ts) * 1000)/1000 #multiply by 1000 to get ms accuracy, divide by 1000 to get x-axis in seconds
	Feedbacktimestamps.append(ts)
	
	RhoE.append(float(rhoE))
	ThetaE.append(float(thetaE))
	curVex = float(rhoE)*np.cos(float(thetaE))
	curVey = float(rhoE)*np.sin(float(thetaE))
	VexLocal.append(curVex)
	VeyLocal.append(curVey)
	if dribblerSeesBall == 'False': DribblerSeesBall.append(0.0)
	else: DribblerSeesBall.append(1000.0)
	if ballSensorWorking == 'False': BallSensorWorking.append(0.0)
	else: BallSensorWorking.append(1.0)

for i in range(len(VexLocal)):
	VexGlobal.append(VexLocal[i]*np.cos(XsensYaw[i]) + VeyLocal[i]*np.sin(XsensYaw[i]))
	VeyGlobal.append(-VexLocal[i]*np.sin(XsensYaw[i]) + VeyLocal[i]*np.cos(XsensYaw[i]))

#Shift the time values to make it start at 0
firstTimeVal = min(Feedbacktimestamps[0], Statetimestamps[0], Commandedtimestamps[0]) # find the lowest timestamp
for i in range(len(Feedbacktimestamps)):
	Feedbacktimestamps[i] -= firstTimeVal
for i in range(len(Statetimestamps)):
	Statetimestamps[i] -= firstTimeVal
for i in range(len(Commandedtimestamps)):
	Commandedtimestamps[i] -= firstTimeVal

if plotID == "bode":
	linesC = open('PIDfiles/velxCommand.csv', 'r').readlines()
	linesM = open('PIDfiles/velxMeasured.csv', 'r').readlines()
	tcom = [float(ln.replace('\n','').split(', ')[0]) for ln in linesC]
	vcom = [float(ln.replace('\n','').split(', ')[1]) for ln in linesC]
	tmeas = [float(ln.replace('\n','').split(', ')[0]) for ln in linesM]
	vmeas = [float(ln.replace('\n','').split(', ')[1]) for ln in linesM]

	fig, ax = plt.subplots()
	ax.plot(tcom, vcom)
	ax.plot(tmeas, vmeas)
	ax.set_title("Velocity in x-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()

if plotID == "x":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, Vx)
	ax.plot(Feedbacktimestamps, VexLocal)
	ax.plot(Feedbacktimestamps, VexGlobal)
	ax.set_title("Velocity in x-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated Local", "Estimated Global"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()

if plotID == "rho":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, Rho)
	ax.plot(Feedbacktimestamps, RhoE)
	ax.set_title("Magnitude of movement", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()

if plotID == "theta":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, Theta)
	ax.plot(Feedbacktimestamps, ThetaE)
	ax.set_title("Direction of movement", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()
	
if plotID == "y":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, Vy)
	ax.plot(Feedbacktimestamps, VeyLocal)
	ax.plot(Feedbacktimestamps, VeyGlobal)
	ax.set_title("Velocity in y-direction", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated Local", "Estimated Global"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [m/s]", fontsize = "x-large")
	plt.show()
	
if plotID == "w":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, AngleVel)
	ax.plot(Statetimestamps, rateOfTurn)
	ax.set_title("Angular velocity", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Velocity [rad/s]", fontsize = "x-large")
	plt.show()
	
if plotID == "yaw":
	fig, ax = plt.subplots()
	
	ax.plot(Commandedtimestamps, Angle)
	ax.plot(Statetimestamps, XsensYaw)
	ax.set_title("Absolute angle", fontsize = "xx-large")
	ax.legend(["Commanded", "Estimated"])
	ax.set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Angle [rad]", fontsize = "x-large")
	plt.show()

if plotID == "dribbler":
	fig, ax = plt.subplots()
	ax.plot(Statetimestamps, DribbleSpeed)
	ax.plot(Statetimestamps, FilteredDribbleSpeed)
	ax.plot(Statetimestamps, DribbleSpeedBeforeGotBall)
	ax.plot(Feedbacktimestamps, DribblerSeesBall)
	ax.set_title("Absolute angle", fontsize = "xx-large")
	ax.legend(["Dribbler speed", "filtered dribbler speed", "speedBeforeGotBall", "Dribbler sees ball"])
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Angle [rad]", fontsize = "x-large")
	plt.show()

if plotID == "ballsensor":
	fig, ax = plt.subplots()
	
	ax.plot(Feedbacktimestamps, BallSensorWorking)
	ax.set_title("Absolute angle", fontsize = "xx-large")
	ax.legend(["Estimated", "hasBall", "Filtered", "speedBeforeGotBall"])
	ax.set_xlabel("Time [s]", fontsize = "x-large")
	ax.set_ylabel("Angle [rad]", fontsize = "x-large")
	plt.show()


if plotID == "wheels":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Wheel velocities", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, Wc1)
	ax[0, 0].plot(Feedbacktimestamps, W1)
	ax[0, 0].set_title('RF')
	ax[0, 0].legend(["Commanded", "Estimated"])
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[0, 1].plot(Feedbacktimestamps, Wc2)
	ax[0, 1].plot(Feedbacktimestamps, W2)
	ax[0, 1].set_title('RB')
	ax[0, 1].legend(["Commanded", "Estimated"])
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 0].plot(Feedbacktimestamps, Wc3)
	ax[1, 0].plot(Feedbacktimestamps, W3)
	ax[1, 0].set_title('LB')
	ax[1, 0].legend(["Commanded", "Estimated"])
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 1].plot(Feedbacktimestamps, Wc4)
	ax[1, 1].plot(Feedbacktimestamps, W4)
	ax[1, 1].set_title('LF')
	ax[1, 1].legend(["Commanded", "Estimated"])
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Velocity [m/s]', va='center', rotation='vertical', fontsize = "x-large")

	plt.show()

if plotID == "integral":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Integral values from PIDs", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, XInt)
	ax[0, 0].set_title('X')
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[0, 1].plot(Feedbacktimestamps, YInt)
	ax[0, 1].set_title('Y')
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 0].plot(Feedbacktimestamps, WInt)
	ax[1, 0].set_title('W')
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 1].plot(Feedbacktimestamps, YawInt)
	ax[1, 1].set_title('Yaw')
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Integral value', va='center', rotation='vertical', fontsize = "x-large")
	plt.show()
	
if plotID == "integral-wheels":
	fig, ax = plt.subplots(2, 2)
	fig.suptitle("Integral values from wheel PIDs", fontsize = "xx-large")
	ax[0, 0].plot(Feedbacktimestamps, W1Int)
	ax[0, 0].set_title('RF')
	ax[0, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[0, 1].plot(Feedbacktimestamps, W2Int)
	ax[0, 1].set_title('RB')
	ax[0, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 0].plot(Feedbacktimestamps, W3Int)
	ax[1, 0].set_title('LB')
	ax[1, 0].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	ax[1, 1].plot(Feedbacktimestamps, W4Int)
	ax[1, 1].set_title('LF')
	ax[1, 1].set_xticks(np.arange(Feedbacktimestamps[0], Feedbacktimestamps[-1], step=1))
	
	fig.text(0.5, 0.04, "Time [s]", ha="center", fontsize = "x-large")
	fig.text(0.04, 0.5, 'Integral value', va='center', rotation='vertical', fontsize = "x-large")
	plt.show()
