import sys
import os
import socket
import math
from dataclasses import dataclass
from typing import List, Tuple

# Import the required protobuf messages
from roboteam_networking.proto.ssl_simulation_control_pb2 import (
    SimulatorCommand, 
    SimulatorControl,
    TeleportRobot
)
from roboteam_networking.proto.ssl_gc_common_pb2 import Team

SIMULATION_CONTROL_PORT = 10300
ROBOT_RADIUS = 0.09  # in meters

@dataclass
class FieldDimensions:
    width: float = 12.0  # Standard SSL field width in meters
    height: float = 9.0  # Standard SSL field height in meters

class FormationManager:
    def __init__(self, field: FieldDimensions):
        self.field = field
        
    def calculate_defensive_formation_positions(self, num_back: int, num_mid: int, num_front: int,
                                             team_yellow: bool) -> List[Tuple[int, float, float, float]]:
        """Calculate defensive formation positions (closer to our goal)"""
        positions = []
        robot_id = 1
        mirror = 1 if team_yellow else -1
        
        width = self.field.width
        height = self.field.height
        
        # Back line positions (closer together near goal)
        for i in range(num_back):
            x = mirror * (-width / 3)
            y = -height / 8 + height / (num_back + 1) * (i + 1) / 4
            positions.append((robot_id, x, y, math.pi if mirror > 0 else 0))
            robot_id += 1
            
        # Mid line positions
        for i in range(num_mid):
            x = mirror * (-width / 5)
            y = -height / 2 + height / (num_mid + 1) * (i + 1)
            positions.append((robot_id, x, y, math.pi if mirror > 0 else 0))
            robot_id += 1
            
        # Front line positions (closer to our side)
        for i in range(num_front):
            x = mirror * (-ROBOT_RADIUS * 1.5)
            y = -height / 2 + height / (num_front + 1) * (i + 1)
            positions.append((robot_id, x, y, math.pi if mirror > 0 else 0))
            robot_id += 1
            
        return positions

    def calculate_specific_positions(self, team_yellow: bool) -> List[Tuple[int, float, float, float]]:
        """Calculate specific positions for robots with fixed coordinates"""
        positions = []
        mirror = 1 if team_yellow else -1
        
        specific_positions = [
            (0, -6.0, 0.0),    # Seventh robot
            (9, -3.0, 0.1),    # Eighth robot
            (10, -3.0, -0.1)   # Ninth robot
        ]
        
        for robot_id, x, y in specific_positions:
            mirrored_x = mirror * x
            positions.append((robot_id, mirrored_x, y, math.pi if mirror > 0 else 0))
            
        return positions

def teleport_robots_to_positions(positions: List[Tuple[int, float, float, float]], team_yellow: bool):
    """Teleport multiple robots to their positions"""
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    control = SimulatorControl()

    for robot_id, x, y, orientation in positions:
        teleport = TeleportRobot()
        teleport.id.id = robot_id
        teleport.id.team = Team.YELLOW if team_yellow else Team.BLUE
        teleport.x = x
        teleport.y = y
        teleport.orientation = orientation
        teleport.present = True
        teleport.v_x = 0.0
        teleport.v_y = 0.0
        teleport.v_angular = 0.0
        teleport.by_force = False

        robot_teleport = control.teleport_robot.add()
        robot_teleport.CopyFrom(teleport)

    command = SimulatorCommand()
    command.control.CopyFrom(control)
    serialized_command = command.SerializeToString()
    sock.sendto(serialized_command, ("localhost", SIMULATION_CONTROL_PORT))
    sock.close()

def setup_formations():
    """Combined function to setup all formations for both teams"""
    field = FieldDimensions()
    formation_manager = FormationManager(field)
    
    # Yellow team defensive formation (8 front robots)
    yellow_defensive = formation_manager.calculate_defensive_formation_positions(
        num_back=0,
        num_mid=0,
        num_front=8,
        team_yellow=True
    )
    teleport_robots_to_positions(yellow_defensive, team_yellow=True)
    
    # Yellow team specific additional positions
    yellow_specific = formation_manager.calculate_specific_positions(team_yellow=True)
    teleport_robots_to_positions(yellow_specific, team_yellow=True)
    
    # Blue team defensive formation (8 front robots)
    blue_defensive = formation_manager.calculate_defensive_formation_positions(
        num_back=0,
        num_mid=0,
        num_front=8,
        team_yellow=False
    )
    teleport_robots_to_positions(blue_defensive, team_yellow=False)
    
    # Blue team specific additional positions
    blue_specific = formation_manager.calculate_specific_positions(team_yellow=False)
    teleport_robots_to_positions(blue_specific, team_yellow=False)

if __name__ == "__main__":
    # Initialize field dimensions
    field = FieldDimensions()
    formation_manager = FormationManager(field)
    
    print("Setting up combined formations for both teams...")
    
    # Yellow team defensive formation (6 robots)
    yellow_defensive = formation_manager.calculate_defensive_formation_positions(
        num_back=0,
        num_mid=0,
        num_front=8,
        team_yellow=True
    )
    teleport_robots_to_positions(yellow_defensive, team_yellow=True)
    
    # Yellow team specific additional positions (3 robots)
    yellow_specific = formation_manager.calculate_specific_positions(team_yellow=True)
    teleport_robots_to_positions(yellow_specific, team_yellow=True)
    
    # Blue team defensive formation (6 robots)
    blue_defensive = formation_manager.calculate_defensive_formation_positions(
        num_back=0,
        num_mid=0,
        num_front=8,
        team_yellow=False
    )
    teleport_robots_to_positions(blue_defensive, team_yellow=False)
    
    # Blue team specific additional positions (3 robots)
    blue_specific = formation_manager.calculate_specific_positions(team_yellow=False)
    teleport_robots_to_positions(blue_specific, team_yellow=False)
    
    print("Script execution completed")