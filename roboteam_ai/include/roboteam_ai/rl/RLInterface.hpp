#pragma once

#include <zmqpp/zmqpp.hpp>
#include <array>
#include <string>
#include <thread>
#include "utilities/Constants.h"

namespace rtt::ai::stp::rl {

/**
 * @brief Interface class for Reinforcement Learning communication
 * @details Handles ZMQ socket communication and manages game state information.
 *          This class is used to receive RL decisions from the RL model and update the role allocation.
 *          The role allocation for defenders and walls is determined based on the number of attackers.
 */

class RLInterface {
   private:
    // Thread members
    std::thread receiver_thread;
    bool running;

    // Game state members
    int numAttackers;
    int numDefenders;
    int numWallers;
    std::array<bool, 9> gridPositions;  // Array of 9 positions, boolean

    const int MANDATORY_ROLES = 3;  // keeper, passer, receiver
    const int MAX_ROBOTS = static_cast<int>(rtt::ai::constants::MAX_ROBOT_COUNT);

    // ZMQ members
    zmqpp::context_t context;
    zmqpp::socket_t socket;

    /**
     * @brief Processes received string messages and updates the game state
     * @param input_string Format: "N P1 P2 ... P_N" where:
     *                    - N is the number of attackers
     *                    - P1 through PN are positions (1-9) where attackers are located
     */
    void process_string(const std::string& input_string);

    /**
     * @brief Processes string containing only number of attackers
     */
    void process_attackers_string(const std::string& input_string);
    
    /**
     * @brief Starts the receiver thread
     */
    void start();

    /**
     * @brief Allocates roles based on current game state
     * @param requestedAttackers Number of attackers requested
     */
    void allocateRoles(int requestedAttackers);

   public:
    RLInterface();
    ~RLInterface();

    // Getters
    int getNumAttackers() const;
    int getNumDefenders() const;
    int getNumWallers() const;
    std::array<bool, 9> getBinaryOccupancyGrid() const;
    
    // Setters
    void setNumAttackers(int attackers);
    void clearNumAttackers();
    void setNumWallers(int wallers);
    void clearNumWallers();
    void setNumDefenders(int defenders);
    void clearNumDefenders();
    void setBinaryOccupancyGrid(std::array<bool, 9> grid);

    /**
     * @brief Receives and processes full RL decision including grid positions
     */
    void receiveRLDecision();

    /**
     * @brief Receives and processes attacker count only
     */
    void receiveAttackersOnly();

    /**
     * @brief Prints the binary occupancy grid (debug only)
     */
    void printBinaryOccupancyGrid() const;
};

}  // namespace rtt::ai::stp::rl