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
ZMQ_PORT = 5557

def is_kubernetes():
    """Detect if running in Kubernetes environment"""
    return os.getenv('KUBERNETES_SERVICE_HOST') is not None

def create_zmq_socket():
    """Create and connect a ZMQ socket"""
    context = zmq.Context()
    socket = context.socket(zmq.REQ)
    # In Kubernetes, use the service name
    if is_kubernetes():
        socket.connect(f"tcp://roboteam-ray-worker-svc:5557")
    else:
        socket.connect(f"tcp://localhost:5557")
    return context, socket

@dataclass
class FieldDimensions:
    width: float = 12.0  # Standard SSL field width in meters
    height: float = 9.0  # Standard SSL field height in meters

class FormationManager:
    def __init__(self, field: FieldDimensions):
        self.field = field
        
    def calculate_formation_positions(self, team_yellow: bool) -> List[Tuple[int, float, float, float]]:
        """Calculate positions for all robots based on their roles"""
        positions = []
        mirror = 1 if team_yellow else -1
        
        # Define position mappings (robot_id, x, y)
        role_positions = [
            # Front formation
            (1, -0.5, 3.0),    # formation_front_0
            (2, -0.5, 2.0),    # formation_front_1
            (3, -0.6, -0.7),   # formation_front_2
            (4, -0.5, -3.0),   # formation_front_3
            (10, -0.5, -2.0),  # formation_front_4
            
            # Back positions
            (5, -3.0, 1.5),    # leftback
            (6, -3.0, -1.5),   # rightback
            (7, -4.2, 0),      # centerback
            
            # Wingers
            (8, -2.3, 3.5),    # winger_1
            (9, -2.3, -3.5),   # winger_2
        ]
        
        # Create mirrored positions based on team
        for robot_id, x, y in role_positions:
            mirrored_x = mirror * x
            # For blue team (mirror==-1), flip orientation to face the other way
            orientation = math.pi if mirror > 0 else 0
            positions.append((robot_id, mirrored_x, y, orientation))
            
        return positions

def teleport_robots_to_positions(positions: List[Tuple[int, float, float, float]], team_yellow: bool):
    if is_kubernetes():
        context, socket = create_zmq_socket()
        try:
            for robot_id, x, y, orientation in positions:
                command = {
                    "command_type": "teleport_robot",
                    "robot_id": robot_id,
                    "team_yellow": team_yellow,
                    "x": x,
                    "y": y,
                    "orientation": orientation,
                    "v_x": 0.0,
                    "v_y": 0.0,
                    "v_angular": 0.0,
                    "present": True,
                    "by_force": False
                }
                message = json.dumps(command)
                socket.send_string(message)
                reply = socket.recv_string()
        finally:
            socket.close()
            context.term()
    else:
        udp_socket = socket_module.socket(socket_module.AF_INET, socket_module.SOCK_DGRAM)  # Using renamed module
        try:
            control = SimulatorControl()
            for robot_id, x, y, orientation in positions:
                teleport = TeleportRobot()
                teleport.id.id = robot_id
                teleport.id.team = Team.YELLOW if team_yellow else Team.BLUE
                teleport.x = x
                teleport.y = y
                teleport.orientation = orientation
                teleport.v_x = 0.0
                teleport.v_y = 0.0
                teleport.v_angular = 0.0
                teleport.present = True
                teleport.by_force = False

                robot_teleport = control.teleport_robot.add()
                robot_teleport.CopyFrom(teleport)

            command = SimulatorCommand()
            command.control.CopyFrom(control)
            serialized_command = command.SerializeToString()
            udp_socket.sendto(serialized_command, ("localhost", SIMULATION_CONTROL_PORT))
        finally:
            udp_socket.close()

def teleport_robots():
    """Setup formations for both teams"""
    field = FieldDimensions()
    formation_manager = FormationManager(field)
    
    # Yellow team formation
    yellow_positions = formation_manager.calculate_formation_positions(team_yellow=True)
    teleport_robots_to_positions(yellow_positions, team_yellow=True)
    
    # Blue team formation (mirrored)
    blue_positions = formation_manager.calculate_formation_positions(team_yellow=False)
    teleport_robots_to_positions(blue_positions, team_yellow=False)

if __name__ == "__main__":
    print("Setting up formations for both teams...")
    teleport_robots()
    print("Script execution completed")