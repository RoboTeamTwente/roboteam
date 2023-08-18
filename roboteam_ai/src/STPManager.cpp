#include "STPManager.h"

#include <roboteam_utils/Timer.h>
#include <utilities/normalize.h>

#include <chrono>

#include "control/ControlModule.h"
#include "gui/networking/InterfaceGateway.h"
#include "stp/PlayDecider.hpp"
#include "stp/PlayEvaluator.h"
#include "stp/computations/ComputationManager.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "utilities/RuntimeConfig.h"

/**
 * Plays are included here
 */
#include "gui/Out.h"
#include "stp/plays/defensive/DefendPass.h"
#include "stp/plays/defensive/DefendShot.h"
#include "stp/plays/defensive/KeeperKickBall.h"
#include "stp/plays/offensive/Attack.h"
#include "stp/plays/offensive/AttackingPass.h"
#include "stp/plays/offensive/ChippingPass.h"
#include "stp/plays/referee_specific/AggressiveStopFormation.h"
#include "stp/plays/referee_specific/BallPlacementThem.h"
#include "stp/plays/referee_specific/BallPlacementUs.h"
#include "stp/plays/referee_specific/DefensiveStopFormation.h"
#include "stp/plays/referee_specific/FormationPreHalf.h"
#include "stp/plays/referee_specific/FreeKickThem.h"
#include "stp/plays/referee_specific/FreeKickUsAtGoal.h"
#include "stp/plays/referee_specific/FreeKickUsPass.h"
#include "stp/plays/referee_specific/Halt.h"
#include "stp/plays/referee_specific/KickOffThem.h"
#include "stp/plays/referee_specific/KickOffThemPrepare.h"
#include "stp/plays/referee_specific/KickOffUs.h"
#include "stp/plays/referee_specific/KickOffUsPrepare.h"
#include "stp/plays/referee_specific/PenaltyThem.h"
#include "stp/plays/referee_specific/PenaltyThemPrepare.h"
#include "stp/plays/referee_specific/PenaltyUs.h"
#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;
namespace plays = rtt::ai::stp::play;

namespace rtt {

/// Initialize all plays here (since play vector is static, it's better to do it here to make sure it's initialized before use)
const STPManager::PlaysVec STPManager::plays = ([] {
    auto plays = std::vector<std::unique_ptr<ai::stp::Play>>();

    plays.emplace_back(std::make_unique<plays::AttackingPass>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::ChippingPass>());
    plays.emplace_back(std::make_unique<plays::Attack>());
    plays.emplace_back(std::make_unique<plays::Halt>());
    plays.emplace_back(std::make_unique<plays::DefendShot>());
    plays.emplace_back(std::make_unique<plays::DefendPass>());
    plays.emplace_back(std::make_unique<plays::KeeperKickBall>());
    plays.emplace_back(std::make_unique<plays::DefensiveStopFormation>());
    plays.emplace_back(std::make_unique<plays::AggressiveStopFormation>());
    plays.emplace_back(std::make_unique<plays::BallPlacementUs>());
    plays.emplace_back(std::make_unique<plays::BallPlacementThem>());
    // plays.emplace_back(std::make_unique<play::TimeOut>());
    plays.emplace_back(std::make_unique<plays::PenaltyThemPrepare>());
    plays.emplace_back(std::make_unique<plays::PenaltyUsPrepare>());
    plays.emplace_back(std::make_unique<plays::PenaltyThem>());
    plays.emplace_back(std::make_unique<plays::PenaltyUs>());
    plays.emplace_back(std::make_unique<plays::KickOffUsPrepare>());
    plays.emplace_back(std::make_unique<plays::KickOffThemPrepare>());
    plays.emplace_back(std::make_unique<plays::FreeKickThem>());
    plays.emplace_back(std::make_unique<plays::FreeKickUsAtGoal>());
    plays.emplace_back(std::make_unique<plays::FreeKickUsPass>());
    plays.emplace_back(std::make_unique<plays::KickOffUs>());
    plays.emplace_back(std::make_unique<plays::KickOffThem>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::FormationPreHalf>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallRisky>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::ReflectKick>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GenericPass>());
    return plays;
})();

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void STPManager::start(std::atomic_flag &exitApplication) {
    // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
    RTT_INFO("Start looping")
    RTT_INFO("Waiting for field_data and robots...")

    {
        // Set the pointer to world for all plays
        auto const &[_, world] = world::World::instance();
        for (auto &play : plays) {
            play->setWorld(world);
        }
    }

    double accumulator = 0;
    double alpha = 1.0/100.0; // Represents the weight of the current tick duration in the average tick duration ~~ equivalent to about 100 samples
    int lastTickCount = 0;
    int statsUpdateRate = 5;

    roboteam_utils::Timer stpTimer;
    stpTimer.loop(
        [&]() {
            const auto tickDuration = roboteam_utils::Timer::measure([&]() {
                // Tick AI
                runOneLoopCycle();
                tickCounter++;
            }).count();

            stpTimer.limit([&]() {
                if (currentPlay == nullptr) { return; }
                interfaceGateway->publisher()
                    .publishStpStatus(currentPlay, plays, tickCounter)
                    .publishWorld()
                    .publishVisuals();

            }, 45);

            accumulator = alpha * tickDuration + (1 - alpha) * accumulator; // Exponential moving average
            stpTimer.limit(
                [&]() {
                    ai::gui::Out::decimal("Average tick", accumulator, "ms");
                    ai::gui::Out::bounded("FPS", (tickCounter - lastTickCount) * statsUpdateRate, 0, 60, "fps");
                    lastTickCount = tickCounter;
                },
                statsUpdateRate);

            // If this is primary AI, broadcast settings every second
            if (GameSettings::isPrimaryAI()) {
                stpTimer.limit([&]() { io::io.publishSettings(); }, ai::Constants::SETTINGS_BROADCAST_RATE());
            }

            if (exitApplication.test()) {
                stpTimer.stop();
            }
        },
        ai::Constants::STP_TICK_RATE());
}

/// Run everything with regard to behaviour trees
void STPManager::runOneLoopCycle() {
    auto state = io::io.getState();
    if (state.has_field()) {
        if (!fieldInitialized) RTT_SUCCESS("Received first field message!")
        fieldInitialized = true;

        // Note these calls Assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
        auto worldMessage = state.last_seen_world();
        auto fieldMessage = state.field().field();

        std::vector<proto::SSL_WrapperPacket> vision_packets(state.processed_vision_packets().begin(), state.processed_vision_packets().end());
        if (!GameSettings::isLeft()) {
            roboteam_utils::rotate(&worldMessage);
            for (auto &packet : vision_packets) {
                roboteam_utils::rotate(&packet);
            }
        }
        // mainWindow->updateProcessedVisionPackets(vision_packets);

        auto const &[_, world] = world::World::instance();
        world->updateWorld(worldMessage);
        world->updateField(fieldMessage);

        if (!world->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting STP!")
            }
            world->updatePositionControl();
            decidePlay(world);
            robotsInitialized = true;

        } else {
            if (robotsInitialized) {
                RTT_WARNING("No robots found in world. STP is not running")
                robotsInitialized = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else {
        if (fieldInitialized) {
            RTT_WARNING("No field data present!")
            fieldInitialized = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rtt::ai::control::ControlModule::sendAllCommands();
}

void STPManager::decidePlay(world::World *_world, bool ignoreWorldAge) {
    ai::stp::PlayEvaluator::clearGlobalScores();  // reset all evaluations
    ai::stp::ComputationManager::clearStoredComputations();

    /* Check if world is not too old. Can be ignored, when e.g. running the debugger */
    if(!ignoreWorldAge){
        if (ai::Constants::WORLD_MAX_AGE_MILLISECONDS() < rtt::ai::io::io.getStateAgeMs()) {
            RTT_WARNING("World is too old! Age: ", rtt::ai::io::io.getStateAgeMs(), " ms")
            currentPlay = nullptr;
            // Returning here prevents the play from being updated, which means that the play will not be able to send any commands,
            // which means that the robots will not be able to move. This is a safety measure to prevent the robots from moving when the AI is dealing with outdated information.
            return;
        }
    }

    if (!currentPlay || !currentPlay->isValidPlayToKeep() || rtt::ai::RuntimeConfig::ignoreInvariants || ai::stp::PlayDecider::didLockPlay()) {
        // Decide the best play (ignoring the interface play value)
        currentPlay = ai::stp::PlayDecider::decideBestPlay(_world, plays);
        currentPlay->updateField(_world->getField().value());
        currentPlay->initialize();
    } else {
        currentPlay->updateField(_world->getField().value());
    }
    currentPlay->update();
}

STPManager::STPManager(std::shared_ptr<ai::gui::net::InterfaceGateway> interfaceGateway): interfaceGateway(std::move(interfaceGateway)) { }
}  // namespace rtt
