import sys
import os
import zmq
from google.protobuf.message import DecodeError
import numpy as np
import socket
import struct
from . websocketHandler import run_websocket_command


# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto.State_pb2 import State as RoboState  # Alias to avoid overlap
# from roboteam_networking.proto.ssl_gc_state_pb2 import State,TeamInfo  # Alias for referee state
# from roboteam_networking.proto.ssl_gc_api_pb2 import Output as RefereeState  # Alias for referee state
from roboteam_networking.proto.messages_robocup_ssl_referee_pb2 import * # Alias for referee state

IS_IN_K8S = True

def get_zmq_address():
    """Get the appropriate ZMQ address based on environment"""
    if IS_IN_K8S:
        host = "roboteam-ray-worker-svc"
        print("Running in Kubernetes, using service DNS")
    else:
        host = "localhost"
        print("Running locally")
    return f"tcp://{host}:5558"

# Function to get the ball state
def get_ball_state():
   ball_position = np.zeros(2)  # [x, y]
   # Instead of -1, quadrant 4 if ball is in the center
   ball_quadrant = 4

   CENTER_THRESHOLD = 0.01  # Define the center threshold

   context = zmq.Context()
   socket_world = context.socket(zmq.SUB)
   socket_world.setsockopt_string(zmq.SUBSCRIBE, "")
   
   zmq_address = get_zmq_address()
   print(f"Connecting to ZMQ at: {zmq_address}")
   socket_world.connect(zmq_address)

   try:
       print("Waiting for ZMQ message...")
       message = socket_world.recv()
       print("Received ZMQ message")
       state = RoboState.FromString(message)
       
       if not len(state.processed_vision_packets):
           return ball_position, ball_quadrant

       world = state.last_seen_world
       
       if world.HasField("ball"):
           ball_position[0] = world.ball.pos.x
           ball_position[1] = world.ball.pos.y
           
           print("x",ball_position[0])
           print("y",ball_position[1])

           if abs(ball_position[0]) <= CENTER_THRESHOLD and abs(ball_position[1]) <= CENTER_THRESHOLD:
               ball_quadrant = 4  # Center
           elif ball_position[0] < 0:
               ball_quadrant = 0 if ball_position[1] > 0 else 2
           else:
               ball_quadrant = 1 if ball_position[1] > 0 else 3
   except DecodeError:
       print("Failed to decode protobuf message")
   except zmq.ZMQError as e:
       print(f"ZMQ Error: {e}")
   finally:
       socket_world.close()
       context.term()

   return ball_position, ball_quadrant

# Function to get the robot state
def get_robot_state():
    grid_array = np.zeros((4, 2), dtype=int)  # 4 quadrants, 2 columns for yellow/blue counts
    yellow_team_dribbling = False
    blue_team_dribbling = False

    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")

    zmq_address = get_zmq_address()
    print(f"Connecting to ZMQ at: {zmq_address}")
    socket_world.connect(zmq_address)

    try:
        message = socket_world.recv()
        state = RoboState.FromString(message)
        # print(state)

        if not len(state.processed_vision_packets):
            return grid_array, yellow_team_dribbling, blue_team_dribbling

        world = state.last_seen_world

        def get_grid_position(x, y):
            if x < 0:
                return 0 if y > 0 else 2
            else:
                return 1 if y > 0 else 3

        # Process yellow robots
        for bot in world.yellow:
            grid_pos = get_grid_position(bot.pos.x, bot.pos.y)
            grid_array[grid_pos, 0] += 1
            if bot.feedbackInfo.dribbler_sees_ball:
                yellow_team_dribbling = True

        # Process blue robots
        for bot in world.blue:
            grid_pos = get_grid_position(bot.pos.x, bot.pos.y)
            grid_array[grid_pos, 1] += 1
            if bot.feedbackInfo.dribbler_sees_ball:
                blue_team_dribbling = True

    except DecodeError:
        print("Failed to decode protobuf message")
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}")
    finally:
        socket_world.close()
        context.term()

    return grid_array, yellow_team_dribbling, blue_team_dribbling

def get_referee_state():
    multicast_group = '224.5.23.1'
    multicast_port = 10003

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', multicast_port))

    # Join the multicast group
    mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    try:
        # Receive the data
        message, _ = sock.recvfrom(4096)
        referee_state = Referee.FromString(message)

        # Extract information from TeamInfo for Yellow and Blue teams
        yellow_team_info = referee_state.yellow
        blue_team_info = referee_state.blue

        # Get details from Yellow TeamInfo
        yellow_team_data = {
            "name": yellow_team_info.name,
            "score": yellow_team_info.score,
            "red_cards": yellow_team_info.red_cards,
            "yellow_cards": yellow_team_info.yellow_cards,
            "fouls": yellow_team_info.foul_counter,
            "ball_placement_failures": yellow_team_info.ball_placement_failures
        }

        # Get details from Blue TeamInfo
        blue_team_data = {
            "name": blue_team_info.name,
            "score": blue_team_info.score,
            "red_cards": blue_team_info.red_cards,
            "yellow_cards": blue_team_info.yellow_cards,
            "fouls": blue_team_info.foul_counter,
            "ball_placement_failures": blue_team_info.ball_placement_failures
        }

        # Return both the raw referee_state object and a dictionary of extracted details
        if referee_state.HasField('designated_position'):
            designated_position = {
                "x": referee_state.designated_position.x,
                "y": referee_state.designated_position.y
            }
        else:
            designated_position = None

        # Return both the raw referee_state object and a dictionary of extracted details
        return referee_state, {
            "yellow_team": yellow_team_data,
            "blue_team": blue_team_data,
            "stage": referee_state.stage,
            "command": referee_state.command,
            "designated_position": designated_position
        }

    except DecodeError:
        print("Failed to decode referee state message")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()

if __name__ == "__main__":
    # Get robot state
    grid_array, yellow_team_dribbling, blue_team_dribbling = get_robot_state()
    print("Grid-based Robot Array:")
    print(grid_array)
    quadrants = ["Bottom-Left", "Top-Left", "Bottom-Right", "Top-Right"]
    for i, quadrant in enumerate(quadrants):
        print(f"{quadrant}: {grid_array[i, 0]} yellow robots, {grid_array[i, 1]} blue robots")

    # Get both the raw referee_state object and the extracted details
    referee_state, details = get_referee_state()
    
    # Print general referee state information
    print("Referee State Information:")
    print(f"Stage: {details['stage']}")
    print(f"Command: {details['command']}\n")

    # Print Yellow team details
    yellow_team = details["yellow_team"]
    print("Yellow Team:")
    print(f"Name: {yellow_team['name']}")
    print(f"Score: {yellow_team['score']}, Red Cards: {yellow_team['red_cards']}, Yellow Cards: {yellow_team['yellow_cards']}")
    print(f"Fouls: {yellow_team['fouls']}, Ball Placement Failures: {yellow_team['ball_placement_failures']}\n")

    # Print Blue team details
    blue_team = details["blue_team"]
    print("Blue Team:")
    print(f"Name: {blue_team['name']}")
    print(f"Score: {blue_team['score']}, Red Cards: {blue_team['red_cards']}, Yellow Cards: {blue_team['yellow_cards']}")
    print(f"Fouls: {blue_team['fouls']}, Ball Placement Failures: {blue_team['ball_placement_failures']}")

    print(get_referee_state())
