import zmq
import time
from datetime import datetime
import struct

def create_publisher():
    """
    Creates and configures a ZMQ PUB socket for publishing messages
    """
    # Initialize the ZMQ context
    context = zmq.Context()
    
    # Create a PUB socket
    socket = context.socket(zmq.PUB)
    
    # Bind the socket to port 5555
    socket.bind("tcp://*:5555")
    
    return socket


def publish_messages(socket, topic="updates"):
    # Array of integers to send
    numbers = [3, 7, 9, 8]

    while True:
        # Convert numbers to strings
        message = ' '.join(str(num) for num in numbers)
        
        # Send the string message
        socket.send_string(message)
        
        print(f"Sent integers: {numbers}")
        time.sleep(1)

if __name__ == "__main__":
    publisher = create_publisher()
    # Allow time for subscribers to connect
    print("Starting publisher... waiting for subscribers...")
    time.sleep(1)
    publish_messages(publisher)