import sys
import os
import socket
import struct
import time
from google.protobuf.timestamp_pb2 import Timestamp
from roboteam_ai.src.RL.src.getState import get_referee_state

current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

from roboteam_networking.proto.messages_robocup_ssl_referee_pb2 import Referee

REFEREE_CONTROL_PORT = 10003
MULTICAST_GROUP = '224.5.23.1'

def send_halt_command():
    # Step 1: Retrieve the current referee state
    referee_state, _ = get_referee_state()

    # Set command to HALT to stop the game
    referee_state.command = Referee.HALT
    referee_state.command_counter += 1
    referee_state.command_timestamp = int(time.time() * 1e6)

    # Serialize and send the HALT command
    serialized_state = referee_state.SerializeToString()
    print("Sending HALT command")

    # Send the data to the multicast group
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        sock.sendto(serialized_state, (MULTICAST_GROUP, REFEREE_CONTROL_PORT))
        print(f"Sent HALT command to referee on multicast group {MULTICAST_GROUP}, port {REFEREE_CONTROL_PORT}")
    finally:
        sock.close()


def reset_referee_state():
    # Step 1: Retrieve the current referee state
    referee_state, _ = get_referee_state()

    # Step 2: Modify the state to reset it for a new game
    referee_state.match_type = 3
    referee_state.stage = Referee.NORMAL_FIRST_HALF_PRE
    referee_state.command = Referee.PREPARE_KICKOFF_BLUE
    referee_state.command_counter += 1
    referee_state.command_timestamp = int(time.time() * 1e6)  # Update to the current time

    # Reset Blue team information
    blue_team_info = referee_state.blue
    blue_team_info.name = "Blue Team"
    blue_team_info.score = 0
    blue_team_info.red_cards = 0
    blue_team_info.yellow_cards = 0
    blue_team_info.timeouts = 4
    blue_team_info.timeout_time = 5 * 60 * 1000000  # 5 minutes in microseconds
    blue_team_info.ball_placement_failures = 0
    blue_team_info.can_place_ball = True
    blue_team_info.max_allowed_bots = 11
    blue_team_info.goalkeeper = 1

    # Reset Yellow team information
    yellow_team_info = referee_state.yellow
    yellow_team_info.name = "Yellow Team"
    yellow_team_info.score = 0
    yellow_team_info.red_cards = 0
    yellow_team_info.yellow_cards = 0
    yellow_team_info.timeouts = 4
    yellow_team_info.timeout_time = 5 * 60 * 1000000
    yellow_team_info.ball_placement_failures = 0
    yellow_team_info.can_place_ball = True
    yellow_team_info.max_allowed_bots = 11
    yellow_team_info.goalkeeper = 1

    # Set play direction for blue team
    referee_state.blue_team_on_positive_half = True

    # Step 3: Serialize and send the modified referee state
    serialized_state = referee_state.SerializeToString()
    print(referee_state)

    # Send the data to the multicast group
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        sock.sendto(serialized_state, (MULTICAST_GROUP, REFEREE_CONTROL_PORT))
        print(f"Sent reset command to referee on multicast group {MULTICAST_GROUP}, port {REFEREE_CONTROL_PORT}")
    finally:
        sock.close()

if __name__ == "__main__":
    # Step 1: Send HALT command to stop the game
    send_halt_command()

    # Step 2: Reset the referee state to start a new game
    reset_referee_state()
