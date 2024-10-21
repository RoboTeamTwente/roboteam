import sys
import os
import zmq
from google.protobuf.message import DecodeError
import numpy as np

'''
GetRobotState.py is a script to get the state of the robots. It uses the proto:State object.
It returns an array of 20 rows by 2 columns (10 vs 10 excluding keepers) of the current robot positions

Right now it also creates an array of all robot positions but we don't do anything with that.
'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.State_pb2 import State

def get_robot_state():

    # Initialize array
    robot_data = np.zeros((20, 2))  # Array of 20 rows, each filled with xy values

    # Initialize counters
    left_count_yellow = 0
    right_count_yellow = 0
    left_count_blue = 0
    right_count_blue = 0

    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")
    socket_world.connect("tcp://127.0.0.1:5558")

    try:
        message = socket_world.recv()
        state = State.FromString(message)
        
        if not len(state.processed_vision_packets):
            return robot_data, left_count_yellow, right_count_yellow, left_count_blue, right_count_blue
        
        world = state.last_seen_world
        
        # Process yellow robots
        for i in range(10): # loop 10 times
            bot = world.yellow[i+1] # Skip keeper
            robot_data[i] = [bot.pos.x, bot.pos.y]
            if bot.pos.x <= 0:
                left_count_yellow += 1
            else:
                right_count_yellow += 1
        
        # Process blue robots
        for i in range(10): # loop 10 times
            bot = world.blue[i+1] # Skip keeper
            robot_data[i+10] = [bot.pos.x, bot.pos.y]
            if bot.pos.x <= 0:
                left_count_blue += 1
            else:
                right_count_blue += 1

        print(robot_data)
        print(f"Yellow robots: Left - {left_count_yellow}, Right - {right_count_yellow}")
        print(f"Blue robots: Left - {left_count_blue}, Right - {right_count_blue}")
        
    except DecodeError:
        print("Failed to decode protobuf message")
    except zmq.ZMQError as e:
        print(f"ZMQ Error: {e}")
    finally:
        socket_world.close()
        context.term()

    return left_count_yellow, right_count_yellow, left_count_blue, right_count_blue

if __name__ == "__main__":
    get_robot_state()