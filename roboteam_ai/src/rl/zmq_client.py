import zmq

context = zmq.Context()
socket = context.socket(zmq.PULL)
socket.bind("tcp://*:5555")

print("Waiting for messages...")
while True:
    message = socket.recv_string()
    print(f"Received: {message}")