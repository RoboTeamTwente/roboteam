#include <string>
#include <chrono>
#include <thread>
#include <iostream>
#include <zmq.hpp>

int main() 
{
    using namespace std::chrono_literals;

    std::cout << "Starting server..." << std::endl;

    // initialize the zmq context with a single IO thread
    zmq::context_t context{1};

    // construct a REP (reply) socket and bind to interface
    zmq::socket_t socket{context, zmq::socket_type::rep};
    socket.bind("tcp://*:5555");

    std::cout << "Server is running on port 5555..." << std::endl;

    // prepare some static data for responses
    const std::string data{"World"};

    for (;;) 
    {
        zmq::message_t request;

        std::cout << "Waiting for messages..." << std::endl;
        
        // After (warning fixed):
        if (!socket.recv(request, zmq::recv_flags::none)) {
            std::cerr << "Failed to receive message" << std::endl;
            continue;
        }
        std::cout << "Received " << request.to_string() << std::endl;

        // simulate work
        std::this_thread::sleep_for(1s);

        // send the reply to the client
        auto res = socket.send(zmq::buffer(data), zmq::send_flags::none);
        if (!res) {
            std::cerr << "Failed to send response" << std::endl;
            continue;
        }
        std::cout << "Sent response: " << data << std::endl;
    }

    return 0;
}