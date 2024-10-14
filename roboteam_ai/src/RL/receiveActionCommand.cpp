#include <zmq.hpp>
#include <string>
#include <iostream>
#include <vector>
#include "ActionCommand.pb.h"

int main() {
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_SUB);

    std::cout << "Connecting to ActionCommand sender..." << std::endl;
    socket.connect("tcp://localhost:5555");
    socket.setsockopt(ZMQ_SUBSCRIBE, "", 0);

    std::cout << "ActionCommand receiver started. Ctrl+C to exit." << std::endl;

    while (true) {
        zmq::message_t message;
        socket.recv(&message);

        ActionCommand action_command;
        if (!action_command.ParseFromArray(message.data(), message.size())) {
            std::cerr << "Failed to parse ActionCommand." << std::endl;
            continue;
        }

        if (action_command.numrobots_size() != 3) {
            std::cerr << "Received incorrect number of values. Expected 3, got " 
                      << action_command.numrobots_size() << std::endl;
            continue;
        }

        std::cout << "Received: [" 
                  << action_command.numrobots(0) << ", "
                  << action_command.numrobots(1) << ", "
                  << action_command.numrobots(2) << "]" << std::endl;
    }

    return 0;
}