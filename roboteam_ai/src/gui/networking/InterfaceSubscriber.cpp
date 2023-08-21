//
// Created by Martin Miksik on 24/05/2023.
//

#include "gui/networking/InterfaceSubscriber.h"


#include "RobotHubMode.h"
#include "interface/api/Output.h"
#include "stp/PlayDecider.hpp"
#include "utilities/GameSettings.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "utilities/RuntimeConfig.h"
#include "world/World.hpp"

namespace rtt::ai::gui::net {

void InterfaceSubscriber::onMessage(const proto::MsgFromInterface&& message) {

    switch (message.kind_case()) {
        case proto::MsgFromInterface::kSetPlay: {
            const auto& data = message.set_play();
//            ai::GameStateManager::setGameStateFromInterface(data.play_name(), data.ruleset_name(), data.keeper_id());
            GameStateManager::updateInterfaceGameState(data.play_name().c_str());

            interface::Output::setKeeperId(data.keeper_id());
            interface::Output::setRuleSetName(data.ruleset_name());
            stp::PlayDecider::lockPlay(data.play_name());
        } break;
        
        case proto::MsgFromInterface::kSetRuntimeConfig: {
            RuntimeConfig::useReferee = message.set_runtime_config().use_referee();
            RuntimeConfig::ignoreInvariants = message.set_runtime_config().ignore_invariants();
        } break;
        
        case proto::MsgFromInterface::kSetGameSettings: {
            const auto& gameSettings = message.set_game_settings();
            GameSettings::setLeft(gameSettings.is_left());
            GameSettings::setYellow(gameSettings.is_yellow());
            GameSettings::setRobotHubMode(rtt::net::robotHubModeFromProto(gameSettings.robot_hub_mode()));
        } break;
        
        case proto::MsgFromInterface::kPauseAi: {
            RuntimeConfig::isPaused = message.pause_ai();
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