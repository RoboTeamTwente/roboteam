import zmq
import sys
import os
import numpy as np
import time

def send_action_command(action_array):
    """
    Takes a MultiDiscrete array of 6 numbers (0-9 each) and sends it via ZMQ.
    
    Args:
        action_array: numpy array of 6 integers where:
            0 = defender (not included in message)
            1-9 = attacker in that grid position
    
    Format sent: "N P1 P2 ... PN" where:
        N = number of attackers
        P1...PN = positions of attackers (1-9)
    """
    try:
        # Initialize ZMQ context and socket
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:5555")
        
        # Count attackers (non-zero values)
        attacker_positions = [pos for pos in action_array if pos != 0]
        num_attackers = len(attacker_positions)
        
        # Create message string: "N P1 P2 ... PN"
        message = f"{num_attackers}"
        if num_attackers > 0:
            positions_str = " ".join(map(str, attacker_positions))
            message += f" {positions_str}"
            
        # Send message
        socket.send_string(message)
        time.sleep(0.1)  # Small delay to ensure message is sent
        
        # Clean up
        socket.close()
        context.term()
        
    except Exception as e:
        print(f"Error in send_action_command: {e}")

def send_num_attackers(num_attackers):
    """
    Takes a single number representing number of attackers and sends via ZMQ.
    
    Args:
        num_attackers: integer between 0 and 6 representing number of attackers
    """
    try:
        # Initialize ZMQ context and socket
        context = zmq.Context()
        socket = context.socket(zmq.PUB)
        socket.bind("tcp://*:5555")
        
        # Ensure number is within valid range
        num_attackers = max(0, min(num_attackers, 6))
        
        # Send message
        message = str(num_attackers)
        socket.send_string(message)
        time.sleep(0.1)  # Small delay to ensure message is sent
        
        # Clean up
        socket.close()
        context.term()
        
    except Exception as e:
        print(f"Error in send_num_attackers: {e}")

# Example usage:
# Full action array:
# action = [0, 3, 0, 7, 1, 0]  # 3 attackers in positions 3, 7, and 1
# send_action_command(action)  # Sends: "3 3 7 1"

# Just number of attackers:
# send_num_attackers(3)  # Sends: "3"