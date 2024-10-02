import sys
import os
import zmq
from google.protobuf.message import DecodeError
import numpy as np

'''
GetBallState.py is a script to get the state of the ball (xy pos)
This data will be fed into the RL. It uses the proto:State object.
'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Now import the generated protobuf classes
from roboteam_networking.proto.State_pb2 import State

def get_ball_state():

    # Initialize array
    data_array = np.zeros((1, 2)) # Array of 1 row, with xy values

    context = zmq.Context()
    socket_world = context.socket(zmq.SUB)
    socket_world.setsockopt_string(zmq.SUBSCRIBE, "")
    socket_world.connect("tcp://127.0.0.1:5558")

    while True:
        try:
            message = socket_world.recv()
            state = State.FromString(message)
            
            if not len(state.processed_vision_packets):
                continue
            
            world = state.last_seen_world
            
            # Get ball information
            if world.HasField("ball"):
                # Determine which half the ball is in
                if world.ball.pos.x < 0:
                    data_array[0] = [1, 0]  # Left half
                    left_count = 1
                    right_count = 0
                else:
                    data_array[0] = [0, 1]  # Right half
                    left_count = 0
                    right_count = 1
            
            print(data_array)
            
        except DecodeError:
            print("Failed to decode protobuf message")
        except zmq.ZMQError as e:
            print(f"ZMQ Error: {e}")

        return data_array, left_count, right_count

if __name__ == "__main__":
    get_ball_state()