import zmq
import socket
import json
from roboteam_networking.proto.ssl_simulation_control_pb2 import SimulatorCommand, SimulatorControl, TeleportBall, TeleportRobot
from roboteam_networking.proto.ssl_gc_common_pb2 import Team

CONTROL_PORT = 10300
ZMQ_PORT = 5557

def create_sockets(zmq_port=ZMQ_PORT, simulation_port=CONTROL_PORT):
    """Create and return ZMQ and UDP sockets"""
    # ZMQ setup
    context = zmq.Context()
    zmq_socket = context.socket(zmq.REP)
    zmq_socket.bind(f"tcp://*:{zmq_port}")
    
    # UDP setup
    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    simulation_address = ("localhost", simulation_port)
    
    return context, zmq_socket, udp_socket, simulation_address

def handle_teleport_ball(data, control):
    """Handle teleport ball command"""
    teleport = TeleportBall()
    teleport.x = data["x"]
    teleport.y = data["y"]
    teleport.z = data.get("z", 0.0)
    teleport.vx = data.get("vx", 0.0)
    teleport.vy = data.get("vy", 0.0)
    teleport.vz = data.get("vz", 0.0)
    
    control.teleport_ball.CopyFrom(teleport)

def run_server():
    """Run the ZMQ to UDP bridge server"""
    context, zmq_socket, udp_socket, simulation_address = create_sockets()
    print("ZMQ to UDP bridge server running...")
    
    try:
        while True:
            try:
                # Receive ZMQ message
                message = zmq_socket.recv_string()
                data = json.loads(message)
                
                # Create control message
                control = SimulatorControl()
                handle_teleport_ball(data, control)
                
                # Create and send command
                command = SimulatorCommand()
                command.control.CopyFrom(control)
                serialized_command = command.SerializeToString()
                udp_socket.sendto(serialized_command, simulation_address)
                
                # Send acknowledgment back to client
                zmq_socket.send_string("Command processed successfully")
                
            except Exception as e:
                print(f"Error processing command: {e}")
                zmq_socket.send_string(f"Error: {str(e)}")
    finally:
        zmq_socket.close()
        udp_socket.close()
        context.term()

if __name__ == "__main__":
    run_server()