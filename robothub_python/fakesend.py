import time
import socket
import math
import numpy as np
import os

import threading
import multiprocessing as mp
import zmq

import utils
from REMParser import REMParser

from RobotCommands_pb2 import RobotCommands, RobotCommand

print(f"Opening ZMQ for feedback..")
context = zmq.Context()
sock = context.socket(zmq.PUB)
# sock.setsockopt_string(zmq.SUBSCRIBE, "")
sock.bind(f"tcp://127.0.0.1:5560") # Yellow


cmd = RobotCommands()
    
wer = cmd.robot_commands.add()
wer.id = 1

print("Sending")
sock.send(cmd.SerializeToString())
