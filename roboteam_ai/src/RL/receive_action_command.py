import zmq
import sys
import os

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto import ActionCommand_pb2

def receive_action_command():
    context = zmq.Context()
    socket = context.socket(zmq.PULL)  # Use PULL to receive messages
    socket.connect("tcp://localhost:5555")  # Connect to the sender's port.

    while True:
        message = socket.recv()  # Receive the message.

        action_command = ActionCommand_pb2.ActionCommand()
        action_command.ParseFromString(message)  # Deserialize the message.

        # Print the received data
        print(f"Received: numDefender={action_command.numDefender}, "
              f"numAttacker={action_command.numAttacker}, "
              f"numWaller={action_command.numWaller}")

if __name__ == "__main__":
    print("Receiver is running...")
    receive_action_command()
