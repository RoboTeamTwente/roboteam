#include "rl/RLInterface.hpp"
#include <iostream>
#include <string>
#include <sstream>
#include <optional>
#include <array>
#include <thread>
#include <roboteam_utils/Print.h>

namespace rtt::ai::stp::rl {

RLInterface::RLInterface() :
    context(),
    socket(context, zmqpp::socket_type::subscribe),
    running(false)
{
    RTT_INFO("Starting zmq client...");
    try {
        socket.subscribe("");
        socket.connect("tcp://localhost:5555");
        start();
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
                receiveRLDecision();
                // std::this_thread::sleep_for(std::chrono::milliseconds(100));
            } catch (const std::exception& e) {
                RTT_WARNING("Error in receiver thread: " + std::string(e.what()));
            }
        }
    });
}

RLInterface::~RLInterface() {
    running = false;
    socket.close();
    context.terminate();

    // Clean up the thread
    if(receiver_thread.joinable()) {
        receiver_thread.join();
    }
}
int RLInterface::getNumAttackers() const {
    return numAttackers;
}

std::optional<int> RLInterface::getNumDefenders() const {
    return numDefenders;
}

std::optional<int> RLInterface::getNumWallers() const {
    return numWallers;
}

std::array<bool, 9> RLInterface::getBinaryOccupancyGrid() const {
    return gridPositions;
}

void RLInterface::setNumAttackers(int attackers) {
    numAttackers = attackers;
}

void RLInterface::clearNumAttackers() {
    numAttackers = 0;
}

void RLInterface::setNumWallers(int wallers) {
    numWallers = wallers;
}

void RLInterface::clearNumWallers() {
    numWallers = std::nullopt;
}

void RLInterface::setBinaryOccupancyGrid(std::array<bool, 9> grid) {
    gridPositions = grid;
}

void RLInterface::receiveRLDecision() {
    zmqpp::message message;
    socket.receive(message);
    std::string msg_str;
    message >> msg_str;  // Extract string from message
    
    process_string(msg_str);
}

void RLInterface::printBinaryOccupancyGrid() const {
    for (int i = 0; i < 9; i += 3) {
        std::cout << gridPositions[i] << gridPositions[i+1] << gridPositions[i+2] << std::endl;
    }
    std::cout << std::endl;
}

void RLInterface::process_string(const std::string& input_string) {
    std::istringstream stream(input_string);
    stream >> numAttackers;
    std::fill(gridPositions.begin(), gridPositions.end(), false);
    for (int i = 0; i < numAttackers; i++) {
        int position;
        stream >> position;
        gridPositions[position - 1] = true;
    }

    setNumAttackers(numAttackers);
    setBinaryOccupancyGrid(gridPositions);
}

}  // namespace rtt::ai::stp::rl

// // Main function to create and use the class
// int main() {

//     rtt::ai::stp::rl::RLInterface receiver;
    
//     while(true) {
//         //std::this_thread::sleep_for(std::chrono::seconds(2));
//         receiver.receiveRLDecision();
//         std::cout << "numAttackers: " << receiver.getNumAttackers() << std::endl;
//         receiver.printBinaryOccupancyGrid();
//     }
    
//     return 0;
// }