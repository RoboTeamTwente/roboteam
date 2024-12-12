#pragma once

#include <stp/Play.hpp>
#include "gui/networking/InterfaceGateway.h"
#include "rl/RLInterface.hpp"

namespace rtt {

/**
 * @brief Class that describes the STPManager. The STPManager manages the overall strategy of the robots: the plays
 */
class STPManager {
   public:
    using PlaysVec = std::vector<std::unique_ptr<rtt::ai::stp::Play>>;

    /**
     * @brief Constructs the STPManager with an interface
     * @param interfaceGateway The interface that belongs to this AI
     */
    explicit STPManager(std::shared_ptr<ai::gui::net::InterfaceGateway> interfaceGateway);

    /**
     * @brief Provides global access to the RLInterface
     * @return The RLInterface instance
     */
    static rtt::ai::stp::rl::RLInterface& getRLInterface() {  
        return instance->rlInterface; 
    }

    /**
     * @brief Checks if the STPManager instance has been initialized
     * @return true if the instance exists, false otherwise
     */
    static bool isInitialized() {
    return instance != nullptr;
}

   private:
    /**
     * @brief Runs AI for one iteration
     */
    void runOneLoopCycle();

    int tickCounter = 0;                                              /**< Counter that keeps track of the ticks */
    bool fieldInitialized = false;                                    /**< Indicates whether the field is initialized successfully */
    bool robotsInitialized = false;                                   /**< Indicates whether the robots are initialized successfully */
    std::shared_ptr<ai::gui::net::InterfaceGateway> interfaceGateway; /**< pointer to the InterfaceGateway */

    static STPManager* instance;                                      /**< Static pointer to track the current STPManager instance */
    rtt::ai::stp::rl::RLInterface rlInterface;                        /**< Instance of the RLInterface for RL communication */

    static inline ai::stp::Play* currentPlay{nullptr}; /**< Current best play as picked by the playDecider */

    /**
     * @brief Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     * @param ignoreWorldAge Whether to ignore the age of the world
     */
    void decidePlay(world::World* _world, bool ignoreWorldAge = false);

    /**
     * @brief Updates the dynamic plays (AttackingPass, Attack, Defend) with the current world state
     * @details Checks if currentPlay points to a dynamic play and sets it to nullptr if it does.
     *          Then removes old dynamic plays and creates new ones with updated world information.
     * @param _world Pointer to the current world state
     */
    void updateDynamicPlays(world::World* _world);

   public:
    /**
     * @brief Starts the AI with a synchronized boolean to ensure that AI exits correctly
     * @param exitApplication Indicates whether the AI should exit
     */
    void start(std::atomic_flag& exitApplication);

    static PlaysVec plays; /**< The vector that contains all plays */

    /**
     * @brief Delete copy constructor of the STPManager class
     */
    STPManager(STPManager const&) = delete;

    /**
     * @brief delete copy assignment operator
     */
    STPManager& operator=(STPManager const&) = delete;
};

}  // namespace rtt