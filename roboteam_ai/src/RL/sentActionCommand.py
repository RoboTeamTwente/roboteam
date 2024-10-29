import zmq
import sys
import os
import time  # Importing time for sleep

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto import ActionCommand_pb2

def send_action_command(num_attacker, num_defender, num_waller):
    context = zmq.Context()
    socket = context.socket(zmq.PUSH)  # Use PUSH socket type
    socket.bind("tcp://*:5555")  # Send data over this port.

 
    action_command = ActionCommand_pb2.ActionCommand()
    action_command.numDefender = num_defender
    action_command.numAttacker = num_attacker
    action_command.numWaller = num_waller

    message = action_command.SerializeToString()
    socket.send(message) 
    print(f"Sent: numDefender={num_defender}, numAttacker={num_attacker}, numWaller={num_waller}")
    

if __name__ == "__main__":
    # Example usage
    send_action_command(2, 3, 1)
