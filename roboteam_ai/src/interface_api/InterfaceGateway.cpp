//
// Created by Martin Miksik on 25/04/2023.
//

#include "interface_api/InterfaceGateway.h"

#include "RobotHubMode.h"
#include "STPManager.h"
#include "interface_api/RuntimeConfig.h"
#include "proto/NewInterface.pb.h"
#include "stp/PlayDecider.hpp"
#include "utilities/GameSettings.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "utilities/Pause.h"

namespace rtt::ai::io {
InterfaceGateway& InterfaceGateway::instance() {
    static InterfaceGateway instance;
    return instance;
}

void InterfaceGateway::broadcastSTPStatus(stp::Play* selectedPlay, const std::vector<std::unique_ptr<rtt::ai::stp::Play>>& plays, int currentTick) {
    auto envelope = proto::MsgToInterface();
    const auto stpStatus = envelope.mutable_stp_status();
    stpStatus->set_current_tick(currentTick);
    stpStatus->mutable_selected_play()->set_play_name(selectedPlay->getName());
    stpStatus->mutable_selected_play()->set_play_score(selectedPlay->lastScore.value_or(-1));

    for (const auto& play : plays) {
        auto scoredPlay = stpStatus->add_scored_plays();
        scoredPlay->set_play_name(play->getName());
        scoredPlay->set_play_score(play->getLastScore());
    }

    auto robotsMap = stpStatus->mutable_robots();
    for (auto& [role, status] : selectedPlay->getRoleStatuses()) {
        int robotId = role->getCurrentRobot()->get()->getId();
        (*robotsMap)[robotId] = proto::STPStatus::STPRobot();
        auto& robotMsg = robotsMap->at(robotId);
        robotMsg.set_id(role->getCurrentRobot().value()->getId());

        // Role status
        robotMsg.mutable_role()->set_name(role->getName());
        robotMsg.mutable_role()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(status)});

        // Tactic status
        const auto tactic = role->getCurrentTactic();
        if (!tactic) {continue;}
        robotMsg.mutable_tactic()->set_name(tactic->getName());
        robotMsg.mutable_tactic()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(tactic->getStatus())});

        // Skill status
        const auto skill = tactic->getCurrentSkill();
        if (!skill) {continue;}
        robotMsg.mutable_skill()->set_name(skill->getName());
        robotMsg.mutable_skill()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(skill->getStatus())});
    }
    broadcastMessage(envelope);
}

void InterfaceGateway::broadcastWorld() {
    auto envelope = proto::MsgToInterface();
    auto state = io::io.getState();
    envelope.mutable_state()->CopyFrom(state);
    broadcastMessage(envelope);
}

//// r = 114. t = 116. rtt = 11400+1160+116 = 12676
InterfaceGateway::InterfaceGateway() : webSocketServer(12676) {
    webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket> webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp());
        webSocketPtr.lock()->setOnMessageCallback([&, webSocketPtr, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
            RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId());
            if (msg->type == ix::WebSocketMessageType::Open) {
                RTT_INFO("Sending setup message to client");
                onConnection(webSocketPtr.lock());
                return;
            }

            if (msg->type != ix::WebSocketMessageType::Message) {
                RTT_INFO("Received non-message type, ignoring");
                return;
            }

            auto envelop = proto::MsgFromInterface{};
            envelop.ParseFromString(msg->str);
            RTT_INFO(envelop.DebugString());
            onMessage(std::move(envelop));
        });
    });

    std::pair<bool, std::string> res = webSocketServer.listen();
    if (!res.first) {
        RTT_ERROR("Could not start WebSocketServer :", res.second);
        return;
    }

    webSocketServer.start();
}

void InterfaceGateway::onConnection(const std::shared_ptr<ix::WebSocket>& wss) {
    auto envelope = proto::MsgToInterface();
    const auto setupMessage = envelope.mutable_setup_message();
    for (auto& play : rtt::STPManager::plays) {  // TODO: How is(was) this thread safe?
        setupMessage->add_available_plays(play->getName());
    }

    for (const auto& ruleSet : Constants::ruleSets()) {
        setupMessage->add_available_rulesets(ruleSet.title);
    }

    setupMessage->set_is_paused(Pause::isPaused());

    const auto game_settings = setupMessage->mutable_game_settings();
    game_settings->set_robot_hub_mode(modeToProto(GameSettings::getRobotHubMode()));
    game_settings->set_is_left(GameSettings::isLeft());
    game_settings->set_is_yellow(GameSettings::isYellow());

    const auto ai_settings = setupMessage->mutable_ai_settings();
    ai_settings->set_use_referee(new_interface::RuntimeConfig::useReferee);
    ai_settings->set_ignore_invariants(new_interface::RuntimeConfig::ignoreInvariants);

    std::string state_str = envelope.SerializeAsString();
//    RTT_DEBUG("Sending setup message to client");

    wss->sendBinary(state_str);
}

void InterfaceGateway::onMessage(const proto::MsgFromInterface&& message) {
    RTT_INFO(message.DebugString());

    switch (message.kind_case()) {
        case proto::MsgFromInterface::kSetGameState: {
            const auto& data = message.set_game_state();
            ai::GameStateManager::setGameStateFromInterface(data.playname(), data.rulesetname(), data.keeper_id());
            new_interface::RuntimeConfig::interfacePlay.push(data.playname());
        }
            break;
        case proto::MsgFromInterface::kSetAiSettings: {
            new_interface::RuntimeConfig::useReferee = message.set_ai_settings().use_referee();
            new_interface::RuntimeConfig::ignoreInvariants = message.set_ai_settings().ignore_invariants();
        }
            break;
        case proto::MsgFromInterface::kSetGameSettings: {
            const auto& gameSettings = message.set_game_settings();
            GameSettings::setLeft(gameSettings.is_left());
            GameSettings::setYellow(gameSettings.is_yellow());
            GameSettings::setRobotHubMode(net::modeFromProto(gameSettings.robot_hub_mode()));
        }
            break;
        case proto::MsgFromInterface::kStopResume: {
            if (message.stop_resume()) {
                auto const& [_, world] = rtt::world::World::instance();
                Pause::pause(world->getWorld());
            } else {
                Pause::resume();
            }
        }
            break;
        case proto::MsgFromInterface::KIND_NOT_SET:
            RTT_ERROR("Received message with no kind set");
            break;
    }
}

void InterfaceGateway::broadcastMessage(const google::protobuf::Message& message) {
    std::string state_str = message.SerializeAsString();
    for (const auto& client : webSocketServer.getClients()) {
        client->sendBinary(state_str);
    }
}
void InterfaceGateway::broadcastVisuals() {
    rtt::ai::new_interface::Interface::consumeVisualizations([&](const proto::MsgToInterface::VisualizationBuffer& visuals) {
        auto envelope = proto::MsgToInterface();
        envelope.mutable_visualizations()->CopyFrom(visuals);
        broadcastMessage(envelope);
    });
}

}  // namespace rtt::ai::new_interface