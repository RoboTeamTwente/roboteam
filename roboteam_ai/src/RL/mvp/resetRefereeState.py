import sys
import os
import numpy as np
import time
import socket
import struct
import binascii
import random


'''
resetRefereeState.py is a script to reset the referee state (in the game).
With this we can modify the amount of cards, call a time out or force continue for example.

We reset the following:
- The foul cards (yellow, red)
- The score
- Sent a command to initialize kickoff

'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.messages_robocup_ssl_referee_pb2 import Referee

MULTICAST_ADDR = '224.5.23.1'  # Standard SSL referee multicast address
PORT = 10003  # Standard SSL referee port


def reset_team_cards(team_info):
    team_info.red_cards = 0
    team_info.yellow_card_times.clear()
    team_info.yellow_cards = 0

def send_udp(data, addr, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 32)
    sock.sendto(data, (addr, port))

def reset_referee_state():
    referee_message = Referee()

    # Randomly choose between PREPARE_KICKOFF_US and PREPARE_KICKOFF_THEM
    kickoff_command = random.choice([
        Referee.Command.PREPARE_KICKOFF_YELLOW,
        Referee.Command.PREPARE_KICKOFF_BLUE
    ])
    referee_message.command = kickoff_command
    
    # Reset cards for both teams
    reset_team_cards(referee_message.yellow)
    reset_team_cards(referee_message.blue)
    
    # Send via UDP to port 10003
    send_udp(referee_message.SerializeToString(), "localhost", 10003)

# Call this function to reset cards
reset_referee_state()