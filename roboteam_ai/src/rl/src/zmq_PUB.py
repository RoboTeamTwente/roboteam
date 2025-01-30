import zmq
import time

class Publisher:
    def __init__(self):
        """Initialize ZMQ publisher socket"""
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")
        time.sleep(0.5)
        
    def publish_message(self, num_attackers):
        """Publish a message with the number of attackers"""
        # Ensure number is within valid range
        num_attackers = max(0, min(num_attackers, 6))
        
        # Send message
        message = str(num_attackers)
        self.socket.send_string(message)
        #print(f"Sent integer: {message}")
        
    def close(self):
        """Clean up ZMQ resources"""
        if hasattr(self, 'socket'):
            self.socket.close(linger=0)
        if hasattr(self, 'context'):
            self.context.term()

if __name__ == "__main__":
    publisher = Publisher()
    try:
        # while True:
        publisher.publish_message(0)
            # time.sleep(1)
    finally:
        publisher.close()