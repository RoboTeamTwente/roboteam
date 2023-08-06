//
// Created by Martin Miksik on 24/05/2023.
//

#include "interface_api/InterfaceSubscriber.h"

#include "RobotHubMode.h"
#include "interface/api/Output.h"
#include "interface_api/RuntimeConfig.h"
#include "proto/ssl_simulation_config.pb.h"
#include "utilities/GameSettings.h"
#include "utilities/IOManager.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "world/World.hpp"

namespace rtt::ai::io {

void InterfaceSubscriber::onMessage(const proto::MsgFromInterface&& message) {
    RTT_INFO(message.DebugString());

    switch (message.kind_case()) {
        case proto::MsgFromInterface::kSetPlay: {
            const auto& data = message.set_play();
//            ai::GameStateManager::setGameStateFromInterface(data.play_name(), data.ruleset_name(), data.keeper_id());
            GameStateManager::updateInterfaceGameState(data.play_name().c_str());

            interface::Output::setKeeperId(data.keeper_id());
            interface::Output::setRuleSetName(data.ruleset_name());
            new_interface::RuntimeConfig::interfacePlay.push(data.play_name());
        } break;
        
        case proto::MsgFromInterface::kSetRuntimeConfig: {
            new_interface::RuntimeConfig::useReferee = message.set_runtime_config().use_referee();
            new_interface::RuntimeConfig::ignoreInvariants = message.set_runtime_config().ignore_invariants();
        } break;
        
        case proto::MsgFromInterface::kSetGameSettings: {
            const auto& gameSettings = message.set_game_settings();
            GameSettings::setLeft(gameSettings.is_left());
            GameSettings::setYellow(gameSettings.is_yellow());
            GameSettings::setRobotHubMode(net::robotHubModeFromProto(gameSettings.robot_hub_mode()));
        } break;
        
        case proto::MsgFromInterface::kPauseAi: {
            if (message.pause_ai()) {
                auto const& [_, world] = rtt::world::World::instance();
                Pause::pause(world->getWorld());
            } else {
                Pause::resume();
            }
        } break;
        
        case proto::MsgFromInterface::kSetBallPos: {
            const auto& ballPos = message.set_ball_pos();
            proto::SimulationConfiguration configuration;               // Create packet
            configuration.mutable_ball_location()->set_x(ballPos.x());  // Set x
            configuration.mutable_ball_location()->set_y(ballPos.y());  // Set y

            bool sentConfig = io::io.sendSimulationConfiguration(configuration);  // Send packet
            if (!sentConfig) {
                RTT_WARNING("Failed to send Simulation Configuration command. Is this the primary AI?")
            }
        } break;

        case proto::MsgFromInterface::kSimulatorCommand: {
            const SimulatorCommand& command = message.simulator_command();
            sendPacketToSimulator(command);
        } break;

        case proto::MsgFromInterface::KIND_NOT_SET:
            RTT_ERROR("Received message with no kind set");
            break;
    }
}

void InterfaceSubscriber::sendPacketToSimulator(const SimulatorCommand& packet) {
    QByteArray datagram;
    datagram.resize(packet.ByteSizeLong());
    packet.SerializeToArray(datagram.data(), datagram.size());
    auto bytesSent = simulator_socket.writeDatagram(datagram, QHostAddress::LocalHost, 10300);
}

}  // namespace rtt::ai::io