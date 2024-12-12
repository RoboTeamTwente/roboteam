#pragma once

#include <zmqpp/zmqpp.hpp>
#include <array>
#include <optional>
#include <string>

namespace rtt::ai::stp::rl {

/**
 * @brief Interface class for Reinforcement Learning communication
 * @details Handles ZMQ socket communication and manages game state information
 */

class RLInterface {
private:

    // Thread members
    std::thread receiver_thread;
    bool running;

    // Game state members
    int numAttackers;
    std::optional<int> numDefenders;
    std::optional<int> numWallers;
    std::array<bool, 9> gridPositions; // Array of 9 positions, boolean

    // ZMQ members
    zmqpp::context_t context;
    zmqpp::socket_t socket;

    /**
     * @brief Processes received string messages and updates the game state
     * @param input_string Format: "N P1 P2 ... P_N" where:
     *                    - N is the number of attackers
     *                    - P1 through PN are positions (1-9) where attackers are located
     * @details Parses the input string to:
     *          1. Extract number of attackers (first number)
     *          2. Reset the 3x3 grid to empty
     *          3. Mark grid positions as occupied based on subsequent numbers
     *          4. Updates both numAttackers and gridPositions class members
     * @example "2 1 9" results in:
     *          - numAttackers = 2
     *          - grid positions: [true, false, false,
     *                            false, false, false,
     *                            false, false, true]
     */
    void process_string(const std::string& input_string);


    void start();


public:
    // Constructor
    RLInterface();

    // Destructor
    ~RLInterface();

    // Getters
    int getNumAttackers() const;
    std::optional<int> getNumDefenders() const;
    std::optional<int> getNumWallers() const;
    std::array<bool, 9> getBinaryOccupancyGrid() const;
    
    // Setters
    void setNumAttackers(int attackers);
    void clearNumAttackers();
    void setNumWallers(int wallers);
    void clearNumWallers();
    void setBinaryOccupancyGrid(std::array<bool, 9> grid);


    /**
     * @brief This function is used to receive the RL decision, it internally parses it and updates the game state
     * by calling the other functions such as process_string. It also sets the member variables.
     */
    void receiveRLDecision();

    /**
     * @brief Not used, only for debugging
     */
    void printBinaryOccupancyGrid() const;
};

} // namespace rtt::ai::stp::rl