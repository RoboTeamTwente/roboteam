import sys
import os
import zmq
from google.protobuf.message import DecodeError
import numpy as np
import socket
import struct
# from . websocketHandler import run_websocket_command


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

def is_kubernetes():
    """Detect if running in Kubernetes environment"""
    return os.getenv('KUBERNETES_SERVICE_HOST') is not None

def get_zmq_address():
    """Get the appropriate ZMQ address based on environment"""
    if is_kubernetes():
        host = "roboteam-ray-worker-svc"
        #print("Running in Kubernetes, using service DNS")
    else:
        host = "localhost"
        #print("Running locally")
    return f"tcp://{host}:5558"

def get_zmq_address_2():
    """Get the appropriate ZMQ address based on environment"""
    if is_kubernetes():
        host = "roboteam-ray-worker-svc"
        #print("Running in Kubernetes, using service DNS")
    else:
        host = "localhost"
        #print("Running locally")
    return f"tcp://{host}:5559"

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
   #print(f"Connecting to ZMQ at: {zmq_address}")
   socket_world.connect(zmq_address)

   try:
       #print("Waiting for ZMQ message...")
       message = socket_world.recv()
       #print("Received ZMQ message")
       state = RoboState.FromString(message)
       
       if not len(state.processed_vision_packets):
           return ball_position, ball_quadrant

       world = state.last_seen_world
       
       if world.HasField("ball"):
           ball_position[0] = world.ball.pos.x
           ball_position[1] = world.ball.pos.y
           
           #print("x",ball_position[0])
           #print("y",ball_position[1])

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

def get_robot_state():
    # Initialize array for 22 robots (11 per team) with x,y coordinates
    robot_positions = np.zeros(44)  # [yellow1_x, yellow1_y, ..., blue1_x, blue1_y, ...]
    yellow_team_dribbling = False
    blue_team_dribbling = False
    
    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")

    zmq_address = get_zmq_address()
    socket_world.connect(zmq_address)

    try:
        message = socket_world.recv()
        state = RoboState.FromString(message)

        if not len(state.processed_vision_packets):
            return robot_positions, yellow_team_dribbling, blue_team_dribbling

        world = state.last_seen_world

        # Process yellow robots (first 11 robots, indices 0-21)
        for i, bot in enumerate(world.yellow[:11]):
            idx = i * 2
            robot_positions[idx] = bot.pos.x
            robot_positions[idx + 1] = bot.pos.y
            if bot.feedbackInfo.dribbler_sees_ball:
                yellow_team_dribbling = True

        # Process blue robots (last 11 robots, indices 22-43)
        for i, bot in enumerate(world.blue[:11]):
            idx = (i + 11) * 2
            robot_positions[idx] = bot.pos.x
            robot_positions[idx + 1] = bot.pos.y
            if bot.feedbackInfo.dribbler_sees_ball:
                blue_team_dribbling = True

    except DecodeError:
        print("Failed to decode protobuf message")
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}")
    finally:
        socket_world.close()
        context.term()

    return robot_positions, yellow_team_dribbling, blue_team_dribbling

def get_referee_state_old():
    """
    Returns tuple of (yellow_score, blue_score, stage, command, x, y)
    """
    multicast_group = '224.5.23.1'
    multicast_port = 10003

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        sock.bind(('', multicast_port))
        mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        sock.settimeout(5)

        message, _ = sock.recvfrom(4096)
        referee_state = Referee()
        referee_state.ParseFromString(message)

        x = y = 0
        if referee_state.HasField('designated_position'):
            x = referee_state.designated_position.x
            y = referee_state.designated_position.y

        return (
            referee_state.yellow.score,
            referee_state.blue.score,
            referee_state.stage,
            referee_state.command,
            x/1000,
            y/1000
        )

    except (socket.timeout, DecodeError) as e:
        print(f"Referee state error: {e}")
        return 0, 0, 0, 0, 0, 0

    finally:
        sock.close()

def get_referee_state():
    """
    Returns tuple of (yellow_score, blue_score, stage, command, x, y)
    """
    context = zmq.Context()
    socket_ref = context.socket(zmq.SUB)
    socket_ref.setsockopt_string(zmq.SUBSCRIBE, "")  # Match the working pattern
    
    zmq_address = get_zmq_address_2()
    print(f"Connecting to ZMQ address: {zmq_address}")
    socket_ref.connect(zmq_address)

    try:
        # Just receive the message directly like in get_robot_state()
        message = socket_ref.recv()
        referee_state = Referee()
        referee_state.ParseFromString(message)

        x = y = 0
        if referee_state.HasField('designated_position'):
            x = referee_state.designated_position.x
            y = referee_state.designated_position.y

        print("get_referee_state no error")
        return (
            referee_state.yellow.score,
            referee_state.blue.score,
            referee_state.stage,
            referee_state.command,
            x/1000,
            y/1000
        )

    except DecodeError as e:
        print(f"Proto decode error: {e}")
        return 0, 0, 0, 0, 0, 0
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}")
        return 0, 0, 0, 0, 0, 0
    finally:
        socket_ref.close()
        context.term()

if __name__ == "__main__":
    # Get robot state
    grid_array, yellow_team_dribbling, blue_team_dribbling = get_robot_state()
    print("Grid-based Robot Array:")
    print(grid_array)
    quadrants = ["Bottom-Left", "Top-Left", "Bottom-Right", "Top-Right"]
    for i, quadrant in enumerate(quadrants):
        print(f"{quadrant}: {grid_array[i, 0]} yellow robots, {grid_array[i, 1]} blue robots")

    result = get_referee_state()
    print("\nReturned Values:")
    print(f"Yellow Score: {result[0]}")
    print(f"Blue Score: {result[1]}")
    print(f"Stage: {result[2]}")
    print(f"Command: {result[3]}")
    print(f"X Position: {result[4]}")
    print(f"Y Position: {result[5]}")