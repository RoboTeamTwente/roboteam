#pragma once
#include <type_traits>
#include <gtest/gtest_prod.h>

#include <stp/Play.hpp>

#include "interface/widgets/mainWindow.h"

namespace rtt {

class STPManager {
   public:
    explicit STPManager(ai::interface::MainWindow* mainWindow);

   private:
    FRIEND_TEST(STPManagerTest, it_handles_ROS_data);

    void runOneLoopCycle();
    bool fieldInitialized = false;
    bool robotsInitialized = false;
    ai::interface::MainWindow* mainWindow;

    /**
     * Current best play as picked by the playDecider
     */
    ai::stp::Play* currentPlay{nullptr};

    /**
     * Number of times the AI ticked since launch
     */
    int tickCounter = 0;

    /**
     * All the plays that are available to the AI
     */
    template <class... T, class Enable = std::enable_if_t<(... && std::is_base_of_v<ai::stp::Play, T>)>>
    void registerPlays() {
        plays = std::vector<std::unique_ptr<rtt::ai::stp::Play>>{};
        (plays.emplace_back(std::make_unique<T>()),...);
    }

    /**
     * Function that decides whether to change plays given a world and field.
     * @param _world the current world state
     */
    void decidePlay(world::World* _world);

   public:
    void start(std::atomic_bool& exitApplication);

    /**
     * The vector that contains all plays
     */
    static std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;

    STPManager(STPManager const&) = delete;
    STPManager& operator=(STPManager const&) = delete;
};

}  // namespace rtt