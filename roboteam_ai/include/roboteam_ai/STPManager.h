#pragma once

#include <stp/Play.hpp>
#include "interface/widgets/mainWindow.h"

namespace rtt {

/**
 * @brief Class that describes the STPManager. The STPManager manages the overall strategy of the robots: the plays
 */
class STPManager {
   public:
    /**
     * @brief Constructs the STPManager with an interface
     * @param mainWindow The interface that belongs to this AI
     */
    explicit STPManager(ai::interface::MainWindow* mainWindow);

   private:
    /**
     * @brief Runs AI for one iteration
     */
    void runOneLoopCycle();


    int tickCounter = 0; /**< Counter that keeps track of the ticks */
    bool fieldInitialized = false; /**< Indicates whether the field is initialized successfully */
    bool robotsInitialized = false; /**< Indicates whether the robots are initialized successfully */
    ai::interface::MainWindow* mainWindow; /**< Interface window of the AI */

    /**
     *
     */
    ai::stp::Play* currentPlay{nullptr}; /**< Current best play as picked by the playDecider */

    /**
     * @brief Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     * @param ignoreWorldAge Whether to ignore the age of the world
     */
    void decidePlay(world::World* _world, bool ignoreWorldAge = false);

   public:
    /**
     * @brief Starts the AI with a synchronized boolean to ensure that AI exits correctly
     * @param exitApplication Indicates whether the AI should exit
     */
    void start(std::atomic_bool& exitApplication);

    static std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays; /**< The vector that contains all plays */

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