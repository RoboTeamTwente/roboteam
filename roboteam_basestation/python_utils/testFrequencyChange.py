import time
import math
from inspect import getmembers
import utils
import serial
import numpy as np
import re
import argparse
import sys 
import shutil
import multiprocessing

import roboteam_embedded_messages.python.BaseTypes as BaseTypes
from roboteam_embedded_messages.python.RobotCommand import RobotCommand
from roboteam_embedded_messages.python.RobotFeedback import RobotFeedback
from roboteam_embedded_messages.python.BasestationConfiguration import BasestationConfiguration
from roboteam_embedded_messages.python.BasestationGetConfiguration import BasestationGetConfiguration
from roboteam_embedded_messages.python.BasestationSetConfiguration import BasestationSetConfiguration

ROBOT_ID = 0

ROTATE_COMMAND = RobotCommand()
ROTATE_COMMAND.header = BaseTypes.PACKET_TYPE_REM_ROBOT_COMMAND
ROTATE_COMMAND.remVersion = BaseTypes.LOCAL_REM_VERSION
ROTATE_COMMAND.id = ROBOT_ID
ROTATE_COMMAND.dribbler = 1
ROTATE_COMMAND.rho = 0
ROTATE_COMMAND.theta = 0
ROTATE_COMMAND.angle = 0

STOP_COMMAND = RobotCommand()
STOP_COMMAND.header = BaseTypes.PACKET_TYPE_REM_ROBOT_COMMAND
STOP_COMMAND.remVersion = BaseTypes.LOCAL_REM_VERSION
STOP_COMMAND.id = ROBOT_ID
STOP_COMMAND.dribbler = 0
STOP_COMMAND.rho = 0
STOP_COMMAND.theta = 0
STOP_COMMAND.angle = 0

ROBOT_FEEDBACK = RobotFeedback()

CHANGE_TO_YELLOW = BasestationSetConfiguration()
CHANGE_TO_YELLOW.channel = 0
CHANGE_TO_YELLOW.header = BaseTypes.PACKET_TYPE_REM_BASESTATION_SET_CONFIGURATION
CHANGE_TO_YELLOW.remVersion = BaseTypes.LOCAL_REM_VERSION

CHANGE_TO_BLUE = BasestationSetConfiguration()
CHANGE_TO_BLUE.channel = 1
CHANGE_TO_BLUE.header = BaseTypes.PACKET_TYPE_REM_BASESTATION_SET_CONFIGURATION
CHANGE_TO_BLUE.remVersion = BaseTypes.LOCAL_REM_VERSION

GET_CONFIG = BasestationGetConfiguration()
GET_CONFIG.header = BaseTypes.PACKET_TYPE_REM_BASESTATION_GET_CONFIGURATION;

basestation = None
packetHz = 60
tickCounter = 0
periodLength = 300
lastWritten = time.time()

robotConnected = True
totalCommandsSent = 0
totalFeedbackReceived = 0

started = time.time()
switchAfter = 3
stop = 2 * switchAfter

config = BasestationConfiguration()

def channelToString(channel):
    if channel == 0:
        return "yellow"
    elif channel == 1:
        return "blue"
    else:
        return str(channel)

def readBasestation():
    packet_type = basestation.read(1)
    if len(packet_type) == 0:
        return

    packetType = packet_type[0]

    if packetType == BaseTypes.PACKET_TYPE_REM_ROBOT_FEEDBACK:
        packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_REM_ROBOT_FEEDBACK - 1)
        pass
    elif packetType == BaseTypes.PACKET_TYPE_REM_BASESTATION_CONFIGURATION:
        packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_REM_BASESTATION_CONFIGURATION - 1)
        config.decode(packet)
        print("Received channel: " + channelToString(config.channel))
        
    elif packetType == BaseTypes.PACKET_TYPE_REM_ROBOT_STATE_INFO:
        packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_REM_ROBOT_STATE_INFO - 1)
        pass
    elif packetType == BaseTypes.PACKET_TYPE_REM_BASESTATION_LOG:
        packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_REM_BASESTATION_LOG - 1)
        pass
    elif packetType == BaseTypes.PACKET_TYPE_REM_ROBOT_LOG:
        packet = packet_type + basestation.read(BaseTypes.PACKET_SIZE_REM_ROBOT_LOG - 1)
        pass
    else:
        #print(f"Error : Unhandled packet with type {packetType}")
        pass

def sendRotateCommand():
    basestation.write(ROTATE_COMMAND.encode())
def sendStopCommand():
    basestation.write(STOP_COMMAND.encode())

def sendChangeToYellow():
    basestation.write(CHANGE_TO_YELLOW.encode())
def sendChangeToBlue():
    basestation.write(CHANGE_TO_BLUE.encode())

def sendGetConfig():
    basestation.write(GET_CONFIG.encode())

def doTest():
    print("Starting!")
    for i in range(60):
        sendRotateCommand()
        readBasestation()
        time.sleep(0.016)
    print("Stopping!")
    for i in range(60):
        sendStopCommand()
        readBasestation()
        time.sleep(0.016)

def changeFrequencyBlue():
    print("Changing to blue... ")
    for i in range(30):
        sendChangeToBlue()
        readBasestation()
        time.sleep(0.016)
def changeFrequencyYellow():
    print("Changing to yellow...")
    for i in range(30):
        sendChangeToYellow()
        readBasestation()
        time.sleep(0.016)

basestation = utils.openContinuous(timeout=0.001)

def wait(seconds):
    start = time.time()
    while (time.time() - start) < seconds:
        readBasestation()


sendGetConfig()
wait(2)
doTest()
wait(2)

changeFrequencyBlue()

sendGetConfig()
wait(2)
doTest()
wait(2)

changeFrequencyYellow()

sendGetConfig()
wait(2)
doTest()
wait(2)