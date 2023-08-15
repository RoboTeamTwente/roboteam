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

import roboteam_embedded_messages.python.REM_BaseTypes as BaseTypes
from roboteam_embedded_messages.python.REM_RobotCommand import REM_RobotCommand
from roboteam_embedded_messages.python.REM_RobotFeedback import REM_RobotFeedback
from roboteam_embedded_messages.python.REM_Log import REM_Log

def handleREM_LOG(rem_log):
    # Prepend where the packet originates from
    log_from = "[?]  "
    if rem_log.fromBS: log_from = "[BS] "
    if not rem_log.fromPC and not rem_log.fromBS:
        log_from = f"[{str(rem_log.fromRobotId).rjust(2)}] "

    # Get message. Strip possible newline
    message = rem_log.message.strip()
    message = log_from + message

    # Print message on new line
    nwhitespace = os.get_terminal_size().columns - len(message) - 2
    print(f"\r{message}{' ' * nwhitespace}")

print("Warning! DO NOT USE THIS unless you really don't have a choice!")
print("This was hacked together in under an hour at the BUGA 23 because of robothub issues.")
print("It has never been tested outside of that environment and is not guaranteed to work.")
time.sleep(2)

cmd_blue = REM_RobotCommand()
cmd_blue.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
cmd_blue.fromPC = True	
cmd_blue.remVersion = BaseTypes.REM_LOCAL_VERSION
cmd_blue.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND
 
cmd_yellow = REM_RobotCommand()
cmd_yellow.header = BaseTypes.REM_PACKET_TYPE_REM_ROBOT_COMMAND
cmd_yellow.fromPC = True
cmd_yellow.remVersion = BaseTypes.REM_LOCAL_VERSION
cmd_yellow.payloadSize = BaseTypes.REM_PACKET_SIZE_REM_ROBOT_COMMAND 
 
config_blue = {
    'name': 'Blue',
    'port': 5559,
    'rem_packet': cmd_blue
}
config_yellow = {
    'name': 'Yellow',
    'port': 5560,
    'rem_packet': cmd_yellow
}

command_queue = mp.Queue()
feedback_queue = mp.Queue()

def thread_send_to_robot():
    basestation = utils.openContinuous(timeout=0.001)
    parser = REMParser(basestation)
    
    while True:
        time.sleep(0.001)
        
        parser.read() # Read all available bytes
        parser.process() # Convert all read bytes into packets)
    
        while parser.hasPackets():
            packet = parser.getNextPacket()
            # RobotLog gets special treatment since we're interested in ALL logs, not just the last one
            if type(packet) == REM_Log:
                handleREM_LOG(packet)
            # if type(packet) == REM_RobotFeedback:
            #     feedback_queue.put(packet)
            #     print("feedback")

        if not command_queue.empty():
            cmd = command_queue.get()
            basestation.write(cmd)

def thread_write_feedback():
    # TODO fix this
    # print(f"Opening ZMQ for feedback..")
    # context = zmq.Context()
    # sock = context.socket(zmq.PUB)
    # sock.connect(f"tcp://127.0.0.1:5561")
    pass

def thread_listen_to_commands(arg):
    print(f"Opening ZMQ for {arg['name']}..")
    context = zmq.Context()
    sock = context.socket(zmq.SUB)
    sock.setsockopt_string(zmq.SUBSCRIBE, "")
    sock.connect(f"tcp://127.0.0.1:{arg['port']}")
    cmd = arg['rem_packet']
    
    while True:
        commands = sock.recv()    
        # print(f"Received command for {arg['name']}")
        commands = RobotCommands.FromString(commands)
        for proto_command in commands.robot_commands:
            vx, vy = proto_command.velocity_x, proto_command.velocity_y
            rho = math.sqrt(vx**2 + vy**2)
            theta = math.atan2(vy, vx)

            cmd.toRobotId = proto_command.id
            cmd.toColor = 0 if arg['name'] == 'Yellow' else 1
            cmd.rho = min(rho, 4)
            cmd.theta = -theta
            cmd.angle = proto_command.angle
            cmd.angularVelocity = proto_command.angular_velocity
            cmd.cameraAngle = proto_command.camera_angle_of_robot
            cmd.useCameraAngle = proto_command.camera_angle_of_robot_is_set
            cmd.useAbsoluteAngle = proto_command.camera_angle_of_robot_is_set
            cmd.dribbler = proto_command.dribbler_speed
            cmd.doKick = proto_command.kick_type != 0#RobotCommand.KickType.KICK
            # cmd.doChip = proto_command.kick_type == RobotCommand.KickType.CHIP
            cmd.kickAtAngle = proto_command.kick_at_angle
            cmd.kickChipPower = 6#proto_command.kick_speed
            cmd.doForce = True#proto_command.wait_for_ball
            cmd.feedback = proto_command.ignore_packet
            
            command_queue.put(cmd.encode())

basestation = mp.Process(target=thread_send_to_robot)
feedback_thread = mp.Process(target=thread_write_feedback)
blue = mp.Process(target=thread_listen_to_commands, args=(config_blue,))
yellow = mp.Process(target=thread_listen_to_commands, args=(config_yellow,))

feedback_thread.start()
time.sleep(.2)
basestation.start()
time.sleep(.2)
blue.start()
yellow.start()

basestation.join()
blue.join()
yellow.join()