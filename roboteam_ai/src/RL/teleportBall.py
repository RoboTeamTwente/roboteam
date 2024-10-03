import sys
import os
import socket


'''
setBallLocation.py is a script to teleport the ball to a set location

Input: x,y ball position
Returns: teleport ball action

'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto.ssl_simulation_control_pb2 import SimulatorCommand, SimulatorControl, TeleportBall

SIMULATION_CONTROL_PORT = 10300

def teleport_ball(x, y, z=0.0):
    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    # Create the TeleportBall message
    teleport = TeleportBall()
    teleport.x = x
    teleport.y = y
    teleport.z = z

    # Create the SimulatorCommand
    control = SimulatorControl()
    control.teleport_ball.CopyFrom(teleport)
    command = SimulatorCommand()
    command.control.CopyFrom(control)

    # Serialize and send the command
    serialized_command = command.SerializeToString()
    sock.sendto(serialized_command, ("localhost", SIMULATION_CONTROL_PORT))

    print(f"Sent command to teleport ball to ({x}, {y}, {z})")

    # Close the socket
    sock.close()

if __name__ == "__main__":

    teleport_ball(400, 4000)
    print("Script execution completed")