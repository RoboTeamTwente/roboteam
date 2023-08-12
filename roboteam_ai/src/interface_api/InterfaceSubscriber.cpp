//
// Created by Martin Miksik on 24/05/2023.
//

#include "interface_api/InterfaceSubscriber.h"
#include <QtNetwork>
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
            RTT_WARNING("TODO: Remove kSetBallPos from MsgFromInterface")
        } break;

        case proto::MsgFromInterface::kSimulatorCommand: {
            const SimulatorCommand& command = message.simulator_command();
            rtt::ai::io::io.sendPacketToSimulator(command);
        } break;

        case proto::MsgFromInterface::KIND_NOT_SET:
            RTT_ERROR("Received message with no kind set");
            break;
    }
}

}  // namespace rtt::ai::io