import sys
import os
import socket
import zmq
import json

'''
setBallLocation.py is a script to teleport the ball to a set location

Input: x,y ball position
Returns: teleport ball action
'''

# Make sure to go back to the main roboteam directory
current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

# Import the generated protobuf classes
from roboteam_networking.proto.ssl_simulation_control_pb2 import SimulatorCommand, SimulatorControl, TeleportBall

SIMULATION_CONTROL_PORT = 10300
ZMQ_PORT = 5557

def is_kubernetes():
    """Detect if running in Kubernetes environment"""
    return os.getenv('KUBERNETES_SERVICE_HOST') is not None

def send_zmq_command(x, y, z):
    """Send command via ZMQ"""
    context = None
    socket = None
    
    try:
        context = zmq.Context()
        socket = context.socket(zmq.REQ)
        
        # Connect to appropriate endpoint
        endpoint = f"tcp://roboteam-ray-worker-svc:{ZMQ_PORT}" if is_kubernetes() else f"tcp://localhost:{ZMQ_PORT}"
        socket.connect(endpoint)
        
        # Create and send command
        command = {
            "command_type": "teleport_ball",
            "x": x,
            "y": y,
            "z": z,
            "vx": 0.0,
            "vy": 0.0,
            "vz": 0.0
        }
        message = json.dumps(command)
        socket.send_string(message)
        
        # Wait for reply
        reply = socket.recv_string()
        # print(f"ZMQ command sent successfully")
        return True
        
    except Exception as e:
        print(f"ZMQ error: {str(e)}")
        return False
        
    finally:
        if socket:
            socket.close()
        if context:
            context.term()

def send_udp_command(x, y, z):
    """Send command via UDP"""
    sock = None
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # Create the TeleportBall message
        teleport = TeleportBall()
        teleport.x = x
        teleport.y = y
        teleport.z = z
        teleport.vx = 0.0
        teleport.vy = 0.0
        teleport.vz = 0.0
        
        # Create the SimulatorCommand
        control = SimulatorControl()
        control.teleport_ball.CopyFrom(teleport)
        command = SimulatorCommand()
        command.control.CopyFrom(control)
        
        # Serialize and send
        serialized_command = command.SerializeToString()
        sock.sendto(serialized_command, ("localhost", SIMULATION_CONTROL_PORT))
        # print(f"UDP command sent successfully")
        return True
        
    except Exception as e:
        print(f"UDP error: {str(e)}")
        return False
        
    finally:
        if sock:
            sock.close()

def teleport_ball(x, y, z=0.0):
    """Teleport the ball to a specific location"""
    try:
        if is_kubernetes():
            success = send_zmq_command(x, y, z)
        else:
            success = send_udp_command(x, y, z)
            
        if success:
            print(f"Ball teleported to position ({x}, {y}, {z})")
        else:
            print("Ball teleportation failed")
            
    except Exception as e:
        print(f"Teleportation error: {str(e)}")
        raise

if __name__ == "__main__":
    try:
        teleport_ball(5.000, 4.300, 0.0)
        print("Script execution completed")
    except Exception as e:
        print(f"Script execution failed: {str(e)}")
        sys.exit(1)