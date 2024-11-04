import sys
import os
import socket
import struct
import time
from google.protobuf.timestamp_pb2 import Timestamp
from roboteam_ai.src.RL.src.getState import get_referee_state

current_dir = os.path.dirname(os.path.abspath(__file__))
roboteam_path = os.path.abspath(os.path.join(current_dir, "..", "..", ".."))

# Add to sys.path
sys.path.append(roboteam_path)

from roboteam_networking.proto.ssl_gc_api_pb2 import Input

def send_reset_match_command(host="0.0.0.0", port=10003):
    """
    Send a reset_match command to the SSL Game Controller
    
    Args:
        host (str): The hostname where the game controller is running
        port (int): The port number the game controller is listening on
    """
    # Create the input message
    input_msg = Input()
    
    # Set the reset_match flag to True
    input_msg.reset_match = True
    
    try:
        # Create a TCP socket
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            # Set a timeout of 5 seconds
            sock.settimeout(5)
            
            # Connect to the game controller
            sock.connect((host, port))
            
            # Serialize the message
            serialized_msg = input_msg.SerializeToString()
            msg_length = len(serialized_msg)
            
            print(f"\nMessage length: {msg_length} bytes")
            print(f"Length bytes: {msg_length.to_bytes(4, byteorder='big').hex()}")
            print(f"Message bytes: {serialized_msg.hex()}")
            
            # Send the message length
            length_bytes = msg_length.to_bytes(4, byteorder='big')
            bytes_sent = sock.send(length_bytes)
            print(f"\nSent {bytes_sent} length bytes")
            
            # Send the actual message
            bytes_sent = sock.send(serialized_msg)
            print(f"Sent {bytes_sent} message bytes")
            
            # Wait for response
            try:
                response = sock.recv(3000)
                if response:
                    print("Reset match command sent and response received")
                else:
                    print("Reset match command sent but no response")
            except socket.timeout:
                print("Reset match command sent but timed out waiting for response")
            
    except ConnectionRefusedError:
        print("Could not connect to the game controller. Is it running?")
    except Exception as e:
        print(f"An error occurred: {e}")

# Example usage
if __name__ == "__main__":
    # You can customize the host and port if needed
    send_reset_match_command()