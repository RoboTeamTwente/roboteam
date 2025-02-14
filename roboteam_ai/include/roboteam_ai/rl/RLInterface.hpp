#pragma once

#include <zmqpp/zmqpp.hpp>
#include <array>
#include <string>
#include <thread>

namespace rtt::ai::stp::rl {

class RLInterface {
   private:
    // Thread members
    std::thread receiver_thread;
    bool running;

    // Game state members
    int numAttackers;

    // ZMQ members
    zmqpp::context_t context;
    zmqpp::socket_t socket;

    bool isActive;

    /**
     * @brief Processes string containing only number of attackers
     */
    void process_attackers_string(const std::string& input_string);
    
    /**
     * @brief Starts the receiver thread
     */
    void start();

   public:
    RLInterface();
    ~RLInterface();

    // Getters
    int getNumAttackers() const;
    bool getIsActive() const { return isActive; }
    
    // Setters
    void setNumAttackers(int attackers);

    /**
     * @brief Receives and processes attacker count only
     */
    void receiveAttackersOnly();
};

}  // namespace rtt::ai::stp::rl