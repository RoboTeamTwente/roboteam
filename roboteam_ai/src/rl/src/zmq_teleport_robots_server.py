import sys
import os
import socket as socket_module
import math
import json
import zmq
from dataclasses import dataclass
from typing import List, Tuple

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..","..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto.ssl_simulation_control_pb2 import SimulatorCommand, SimulatorControl, TeleportRobot
from roboteam_networking.proto.ssl_gc_common_pb2 import Team

SIMULATION_CONTROL_PORT = 10300
ROBOT_RADIUS = 0.09  # in meters
ZMQ_PORT = 5552  # Only used in Kubernetes

def is_kubernetes():
    """Detect if running in Kubernetes environment"""
    return os.getenv('KUBERNETES_SERVICE_HOST') is not None

def create_zmq_socket():
    """Create and bind a ZMQ REP socket"""
    try:
        print("Creating ZMQ context...")
        context = zmq.Context()
        print("Creating REP socket...")
        socket = context.socket(zmq.REP)
        bind_address = "tcp://*:5552"  # Bind to all interfaces
        print(f"Binding to {bind_address}")
        socket.bind(bind_address)
        print("Socket bound successfully")
        return context, socket
    except Exception as e:
        print(f"Failed to create ZMQ socket: {str(e)}")
        raise

def handle_zmq_command(command_str: str):
    """Handle incoming ZMQ command"""
    try:
        command = json.loads(command_str)
        print(f"Received command: {command}")
        
        # Create UDP command for simulator
        udp_socket = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_DGRAM)
        try:
            control = SimulatorControl()
            teleport = TeleportRobot()
            
            # Fill in teleport command from received message
            teleport.id.id = command["robot_id"]
            teleport.id.team = Team.YELLOW if command["team_yellow"] else Team.BLUE
            teleport.x = command["x"]
            teleport.y = command["y"]
            teleport.orientation = command["orientation"]
            teleport.v_x = command["v_x"]
            teleport.v_y = command["v_y"]
            teleport.v_angular = command["v_angular"]
            teleport.present = command["present"]
            teleport.by_force = command["by_force"]

            robot_teleport = control.teleport_robot.add()
            robot_teleport.CopyFrom(teleport)

            command_msg = SimulatorCommand()
            command_msg.control.CopyFrom(control)
            serialized_command = command_msg.SerializeToString()
            udp_socket.sendto(serialized_command, ("localhost", SIMULATION_CONTROL_PORT))
            
            return "ok"
        finally:
            udp_socket.close()
    except Exception as e:
        print(f"Error handling command: {str(e)}")
        return f"error: {str(e)}"

def main():
    print("Starting ZMQ teleport server...")
    context, socket = create_zmq_socket()
    try:
        while True:
            print("Waiting for command...")
            message = socket.recv_string()
            print("Processing command...")
            reply = handle_zmq_command(message)
            socket.send_string(reply)
    except KeyboardInterrupt:
        print("Shutting down...")
    finally:
        socket.close()
        context.term()

if __name__ == "__main__":
    main()