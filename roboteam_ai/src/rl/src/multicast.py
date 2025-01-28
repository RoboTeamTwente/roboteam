import socket
import struct
import sys
import zmq
import time

try:
    # Setup ZMQ publisher
    context = zmq.Context()
    zmq_socket = context.socket(zmq.PUB)
    zmq_socket.bind("tcp://*:5559")
    print("ZMQ publisher started on port 5559", file=sys.stderr)
    
    # Setup multicast receiver
    multicast_group = '224.5.23.1'
    multicast_port = 10003
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', multicast_port))
    mreq = struct.pack("4sl", socket.inet_aton(multicast_group), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
    
    print(f"Receiver waiting on UDP port {multicast_port}...", file=sys.stderr)
    
    while True:
        data, _ = sock.recvfrom(4096)
        # Send directly without multipart
        zmq_socket.send(data)
        time.sleep(0.01)
        
except Exception as e:
    print(f"Error in receiver: {e}", file=sys.stderr)
    sys.exit(1)
finally:
    sock.close()
    zmq_socket.close()
    context.term()