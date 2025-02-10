#include "rl/RLInterface.hpp"
#include "stp/computations/PositionComputations.h"
#include <iostream>
#include <string>
#include <sstream>
#include <array>
#include <thread>
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::rl {

RLInterface::RLInterface() :
    context(),
    socket(context, zmqpp::socket_type::subscribe),
    running(false),
    numAttackers(2),  // Default number of attackers
    isActive(false)
{
    RTT_INFO("Starting zmq client...");
    try {
        socket.subscribe("");
        socket.connect("tcp://localhost:5555");
        start();
        isActive = true;
    } catch (const zmqpp::exception& e) {
        RTT_WARNING("Failed to connect socket");
        throw;
    }
}

void RLInterface::start() {
    running = true;  
    receiver_thread = std::thread([this]() {
        while(running) {
            try {
                receiveAttackersOnly();
                isActive = true;
            } catch (const std::exception& e) {
                RTT_WARNING("Error in receiver thread: " + std::string(e.what()));
                isActive = false;
            }
        }
    });
}

RLInterface::~RLInterface() {
    running = false;
    socket.close();
    context.terminate();

    if(receiver_thread.joinable()) {
        receiver_thread.join();
    }
}

int RLInterface::getNumAttackers() const {
    return numAttackers;
}

void RLInterface::setNumAttackers(int attackers) {
    numAttackers = attackers;
}

void RLInterface::receiveAttackersOnly() {
    zmqpp::message message;
    socket.receive(message);
    std::string msg_str;
    message >> msg_str;  
    
    process_attackers_string(msg_str);
}

void RLInterface::process_attackers_string(const std::string& input_string) {
    std::istringstream stream(input_string);
    stream >> numAttackers;
    numAttackers = std::max(0, numAttackers);
    setNumAttackers(numAttackers);
}

}  // namespace rtt::ai::stp::rl