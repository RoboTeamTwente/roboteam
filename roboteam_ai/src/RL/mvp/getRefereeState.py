import sys
import os
import numpy as np
import time
import socket
import struct
import binascii

'''
getRefereeState.py is a script to get the current state of the referee.

Returns:
- Referee stage (e.g. NORMAL_FIRST_HALF)
- Referee command (e.g. BALL_PLACEMENT_US, STOP, HALT)
- Designated position (to put the ball)
- Scores of the teams

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

def parse_referee_message(data):
    referee = Referee()
    referee.ParseFromString(data)
    return referee

def get_referee_state():
    # Create the socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    
    # Allow multiple sockets to use the same PORT number
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to the port
    sock.bind(('', PORT))
    
    # Tell the operating system to add the socket to the multicast group
    # on all interfaces.
    group = socket.inet_aton(MULTICAST_ADDR)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    print(f"Listening for SSL-Referee commands via UDP multicast on {MULTICAST_ADDR}:{PORT}")

    data, addr = sock.recvfrom(2048)
    referee = parse_referee_message(data)
    
    stage = Referee.Stage.Name(referee.stage)
    command = Referee.Command.Name(referee.command)
    x = referee.designated_position.x if referee.HasField('designated_position') else None
    y = referee.designated_position.y if referee.HasField('designated_position') else None
    yellow_score = referee.yellow.score
    blue_score = referee.blue.score
    
    # Detailed yellow card information
    yellow_yellow_cards = referee.yellow.yellow_cards
    blue_yellow_cards = referee.blue.yellow_cards

    sock.close()
    
    return (x, y, 
            yellow_yellow_cards, 
            blue_yellow_cards, command)

# if __name__ == "__main__":

#     stage, command, x, y, yellow_score, blue_score = get_referee_state()
#     print(f"Stage: {stage}")
#     print(f"Command: {command}")
#     print(f"Designated position: ({x}, {y})")
#     print(f"Yellow team score: {yellow_score}")
#     print(f"Blue team score: {blue_score}")


if __name__ == "__main__":
    (x, y, 
    yellow_yellow_cards, 
    blue_yellow_cards, command) = get_referee_state()

    # print(f"Stage: {stage}")
    print(f"Command: {command}")
    print(f"Designated position: ({x}, {y})")
    # print(f"Yellow team score: {yellow_score}")
    # print(f"Blue team score: {blue_score}")
    print(f"Yellow team yellow cards: {yellow_yellow_cards}")
    print(f"Blue team yellow cards: {blue_yellow_cards}")
