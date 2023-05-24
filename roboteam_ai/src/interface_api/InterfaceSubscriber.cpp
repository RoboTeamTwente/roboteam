//
// Created by Martin Miksik on 24/05/2023.
//

#include "interface_api/InterfaceSubscriber.h"

#include "RobotHubMode.h"
#include "interface_api/RuntimeConfig.h"
#include "utilities/GameSettings.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/Pause.h"
#include "world/World.hpp"

namespace rtt::ai::io {

void InterfaceSubscriber::onMessage(const proto::MsgFromInterface&& message) {
    RTT_INFO(message.DebugString());

    switch (message.kind_case()) {
        case proto::MsgFromInterface::kSetGameState: {
            const auto& data = message.set_game_state();
            ai::GameStateManager::setGameStateFromInterface(data.playname(), data.rulesetname(), data.keeper_id());
            new_interface::RuntimeConfig::interfacePlay.push(data.playname());
        } break;
        case proto::MsgFromInterface::kSetAiSettings: {
            new_interface::RuntimeConfig::useReferee = message.set_ai_settings().use_referee();
            new_interface::RuntimeConfig::ignoreInvariants = message.set_ai_settings().ignore_invariants();
        } break;
        case proto::MsgFromInterface::kSetGameSettings: {
            const auto& gameSettings = message.set_game_settings();
            GameSettings::setLeft(gameSettings.is_left());
            GameSettings::setYellow(gameSettings.is_yellow());
            GameSettings::setRobotHubMode(net::robotHubModeFromProto(gameSettings.robot_hub_mode()));
        } break;
        case proto::MsgFromInterface::kStopResume: {
            if (message.stop_resume()) {
                auto const& [_, world] = rtt::world::World::instance();
                Pause::pause(world->getWorld());
            } else {
                Pause::resume();
            }
        } break;
        case proto::MsgFromInterface::KIND_NOT_SET:
            RTT_ERROR("Received message with no kind set");
            break;
    }
}

}  // namespace rtt::ai::io