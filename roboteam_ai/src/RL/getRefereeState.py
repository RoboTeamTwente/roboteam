import os
import sys
import socket
import struct
import binascii
from google.protobuf.json_format import MessageToJson

'''
getRefereeState.py is a script to get state of the referee. 
This includes the current command, designed position for ball placement, and the score for both teams.
'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.ssl_gc_referee_message_pb2 import Referee
from roboteam_networking.proto.ssl_gc_game_event_pb2 import GameEvent

MULTICAST_GROUP = '224.5.23.1'
MULTICAST_PORT = 10003

def get_referee_state():
    # Create the socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    # Bind to the server address
    sock.bind(('', MULTICAST_PORT))
    
    # Tell the operating system to add the socket to the multicast group
    group = socket.inet_aton(MULTICAST_GROUP)
    mreq = struct.pack('4sL', group, socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    print(f"Listening for Referee messages on {MULTICAST_GROUP}:{MULTICAST_PORT}")
    
    command = None
    pos_x = None
    pos_y = None
    yellow_score = None
    blue_score = None
    
    try:
        data, _ = sock.recvfrom(4096)  # Increased buffer size to 4096 bytes
        referee = Referee()
        referee.ParseFromString(data)
        command = Referee.Command.Name(referee.command)
        
        if referee.HasField('designated_position'):
            pos_x = referee.designated_position.x
            pos_y = referee.designated_position.y
        
        yellow_score = referee.yellow.score
        blue_score = referee.blue.score
    except Exception as e:
        print(f"Error parsing message: {e}")
    finally:
        sock.close()
    
    return command, pos_x, pos_y, yellow_score, blue_score

if __name__ == "__main__":
    command, pos_x, pos_y, yellow_score, blue_score = get_referee_state()
    print(f"Command: {command}")
    print(f"Designated Position: ({pos_x}, {pos_y})")
    print(f"Yellow Team Score: {yellow_score}")
    print(f"Blue Team Score: {blue_score}")