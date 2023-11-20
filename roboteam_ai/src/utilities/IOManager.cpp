#include "utilities/IOManager.h"

#include <QtNetwork>
#include <algorithm>
#include <chrono>
#include <roboteam_utils/RobotCommands.hpp>

#include "RobotHubMode.h"
#include "proto/GameSettings.pb.h"
#include "proto/ssl_simulation_config.pb.h"
#include "utilities/GameSettings.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/RuntimeConfig.h"
#include "utilities/normalize.h"
#include "world/World.hpp"

namespace rtt::ai::io {

IOManager io;

bool IOManager::init(bool isPrimaryAI) {
    RTT_INFO("Setting up IO networkers as ", isPrimaryAI ? "Primary" : "Secondary", " AI")
    bool success = true;

    auto worldCallback = std::bind(&IOManager::handleState, this, std::placeholders::_1);
    this->worldSubscriber = std::make_unique<rtt::net::WorldSubscriber>(worldCallback);

    if (isPrimaryAI) {
        try {
            this->settingsPublisher = std::make_unique<rtt::net::SettingsPublisher>();
        } catch (const zmqpp::zmq_internal_exception& e) {
            success = false;
            RTT_ERROR("Failed to open settings publisher channel. Is it already taken?")
        }
    } else {
        try {
            this->settingsSubscriber =
                std::make_unique<rtt::net::SettingsSubscriber>([&](const proto::GameSettings& settings) { GameSettings::handleSettingsFromPrimaryAI(settings); });
        } catch (const zmqpp::zmq_internal_exception& e) {
            success = false;
            RTT_ERROR("Failed to open settings subscriber channel")
        }
    }
    return success;
}

//////////////////////
/// PROTO HANDLERS ///
//////////////////////
void IOManager::handleState(const proto::State& stateMsg) {
    std::unique_lock<std::mutex> lock(stateMutex);  // write lock
    this->state.CopyFrom(stateMsg);

    if (state.has_last_seen_world() and state.last_seen_world().time() != stateWorldLastTimestamp) {
        stateWorldLastTimestamp = state.last_seen_world().time();
        this->stateTimeMs = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    }

    if (state.has_referee()) {
        // Our name as specified by ssl-refbox : https://github.com/RoboCup-SSL/ssl-refbox/blob/master/referee.conf
        std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
        if (state.referee().yellow().name() == ROBOTEAM_TWENTE) {
            GameSettings::setYellow(true);
        } else if (state.referee().blue().name() == ROBOTEAM_TWENTE) {
            GameSettings::setYellow(false);
        }
        if (GameSettings::isPrimaryAI()) {
            GameSettings::setLeft(!(state.referee().blue_team_on_positive_half() ^ GameSettings::isYellow()));
        }
        if (!GameSettings::isLeft()) roboteam_utils::rotate(state.mutable_referee());
        auto const& [_, data] = World::instance();

        ai::GameStateManager::setRefereeData(state.referee(), data);

        // TODO: Fix for new GameStateManager
        // ai::GameStateManager::setGameStateFromReferee(state.referee(), data->getWorld());
    }
}

void IOManager::publishSettings() {
    proto::GameSettings protoSetting;

    protoSetting.set_is_primary_ai(GameSettings::isPrimaryAI());
    protoSetting.set_is_left(GameSettings::isLeft());
    protoSetting.set_is_yellow(GameSettings::isYellow());
    protoSetting.set_robot_hub_mode(robotHubModeToProto(GameSettings::getRobotHubMode()));

    if (this->settingsPublisher != nullptr) {
        this->settingsPublisher->publish(protoSetting);
    }
}

void IOManager::addCameraAngleToRobotCommands(rtt::RobotCommands& robotCommands) {
    const auto state = this->getState();
    if (state.has_last_seen_world()) {
        const auto world = getState().last_seen_world();
        const auto robots = rtt::GameSettings::isYellow() ? world.yellow() : world.blue();
        for (auto& robotCommand : robotCommands) {
            for (const auto robot : robots) {
                if (robot.id() == robotCommand.id) {
                    robotCommand.cameraAngleOfRobot = robot.angle();
                    robotCommand.cameraAngleOfRobotIsSet = true;
                }
            }
        }
    }
}

void IOManager::publishAllRobotCommands(rtt::RobotCommands& robotCommands) {
    if (!RuntimeConfig::isPaused && !robotCommands.empty()) {
        this->addCameraAngleToRobotCommands(robotCommands);

        this->publishRobotCommands(robotCommands, GameSettings::isYellow());
    }
}

bool IOManager::publishRobotCommands(const rtt::RobotCommands& aiCommand, bool forTeamYellow) {
    bool sentCommands = false;

    if (forTeamYellow && this->robotCommandsYellowPublisher != nullptr) {
        sentCommands = this->robotCommandsYellowPublisher->publish(aiCommand) > 0;
    } else if (!forTeamYellow && this->robotCommandsBluePublisher != nullptr) {
        sentCommands = this->robotCommandsBluePublisher->publish(aiCommand) > 0;
    }

    if (!sentCommands) {
        // TODO: Uncomment
        RTT_ERROR("Failed to send command: Publisher is not initialized (yet)");
    }

    return sentCommands;
}

proto::State IOManager::getState() {
    std::lock_guard<std::mutex> lock(stateMutex);  // read lock
    proto::State copy = state;
    return copy;
}

uint64_t IOManager::getStateTimeMs() { return stateTimeMs; }

uint64_t IOManager::getStateAgeMs() { return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - stateTimeMs; }

bool IOManager::obtainTeamColorChannel(bool toYellowChannel) {
    bool obtainedChannel;

    if (toYellowChannel) {
        obtainedChannel = this->robotCommandsYellowPublisher != nullptr;

        // If we do not have the channel yet
        if (!obtainedChannel) {
            try {
                this->robotCommandsYellowPublisher = std::make_unique<rtt::net::RobotCommandsYellowPublisher>();
                this->robotCommandsBluePublisher = nullptr;
                obtainedChannel = true;
            } catch (const zmqpp::zmq_internal_exception& e) {
                this->robotCommandsYellowPublisher = nullptr;
            }
        }
    } else {
        obtainedChannel = this->robotCommandsBluePublisher != nullptr;

        if (!obtainedChannel) {
            try {
                this->robotCommandsBluePublisher = std::make_unique<rtt::net::RobotCommandsBluePublisher>();
                this->robotCommandsYellowPublisher = nullptr;
                obtainedChannel = true;
            } catch (const zmqpp::zmq_internal_exception& e) {
                this->robotCommandsBluePublisher = nullptr;
            }
        }
    }

    return obtainedChannel;
}

void IOManager::sendPacketToSimulator(const SimulatorCommand& packet) {
    std::unique_lock<std::mutex> lock(simulator_socket_mutex);
    QByteArray datagram;
    datagram.resize(packet.ByteSizeLong());
    packet.SerializeToArray(datagram.data(), datagram.size());
    simulator_socket.writeDatagram(datagram, QHostAddress::LocalHost, 10300);
}

}  // namespace rtt::ai::io
