import sys
import os
import zmq
from google.protobuf.message import DecodeError
import numpy as np

'''
GetState.py is a script to get the state of the game (where the robots and ball are and more information).
This data will be fed into the RL. It uses the proto:State object.
The two functions get_ball_state and get_robot_state get the states of the ball and robots respectively.
'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.State_pb2 import State

def get_ball_state():
    # Initialize array for ball position
    ball_position = np.zeros(2)  # [x, y]
    ball_quadrant = -1  # Initialize to invalid quadrant

    # Define threshold for center position
    CENTER_THRESHOLD = 0.01  # Adjust this value as needed

    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")
    socket_world.connect("tcp://127.0.0.1:5558")

    try:
        message = socket_world.recv()
        state = State.FromString(message)
        
        if not len(state.processed_vision_packets):
            return ball_position, ball_quadrant
        
        world = state.last_seen_world
        
        # Get ball information
        if world.HasField("ball"):
            ball_position[0] = world.ball.pos.x
            ball_position[1] = world.ball.pos.y
            
            # Determine which quadrant the ball is in
            # Assuming the field is centered at (0,0) and extends from -6 to 6 in x and -4.5 to 4.5 in y
            if abs(ball_position[0]) <= CENTER_THRESHOLD and abs(ball_position[1]) <= CENTER_THRESHOLD:
                ball_quadrant = 4  # Center
            elif ball_position[0] < 0: # If left
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

    # Initialize the grid vector
    # 4 positions (2x2 grid), each with counts for yellow and blue robots
    grid_vector = np.zeros(8, dtype=int)

    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")
    socket_world.connect("tcp://127.0.0.1:5558")

    try:
        message = socket_world.recv()
        state = State.FromString(message)
        
        if not len(state.processed_vision_packets):
            return grid_vector
        
        world = state.last_seen_world
        
        def get_grid_position(x, y):
            # Assuming the field is centered at (0,0) and extends from -6 to 6 in x and -4.5 to 4.5 in y
            # Adjust these values if your field dimensions are different
            if x < 0:
                return 0 if y < 0 else 1
            else:
                return 2 if y < 0 else 3
        
        # Process yellow robots (excluding keeper)
        for i in range(1, 11):  # Start from 1 to exclude keeper
            bot = world.yellow[i]
            grid_pos = get_grid_position(bot.pos.x, bot.pos.y)
            grid_vector[grid_pos * 2] += 1
        
        # Process blue robots (excluding keeper)
        for i in range(1, 11):  # Start from 1 to exclude keeper
            bot = world.blue[i]
            grid_pos = get_grid_position(bot.pos.x, bot.pos.y)
            grid_vector[grid_pos * 2 + 1] += 1

    except DecodeError:
        print("Failed to decode protobuf message")
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}")
    finally:
        socket_world.close()
        context.term()

    return grid_vector

if __name__ == "__main__":
    # vector = get_robot_state()
    # print("Grid-based Robot Vector:")
    # print(vector)
    # print("\nInterpretation:")
    # quadrants = ["Bottom-Left", "Top-Left", "Bottom-Right", "Top-Right"]
    # for i in range(4):
    #     print(f"{quadrants[i]}: {vector[i*2]} yellow robots, {vector[i*2+1]} blue robots")

    position, quadrant = get_ball_state()
    print(f"Ball position: ({position[0]:.2f}, {position[1]:.2f})")
    print(f"Ball quadrant: {quadrant}")
    quadrant_names = ["Bottom-Left", "Top-Left", "Bottom-Right", "Top-Right", "Center"]
    if 0 <= quadrant < 5:
        print(f"Ball is in the {quadrant_names[quadrant]}")
    else:
        print("Ball position is invalid or not on the field")