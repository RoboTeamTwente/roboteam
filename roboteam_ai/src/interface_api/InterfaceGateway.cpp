//
// Created by Martin Miksik on 25/04/2023.
//

#include "interface_api/InterfaceGateway.h"

#include <memory>

#include "RobotHubMode.h"
#include "STPManager.h"
#include "interface_api/InterfaceSubscriber.h"
#include "interface_api/RuntimeConfig.h"
#include "utilities/GameSettings.h"
#include "utilities/Pause.h"

namespace rtt::ai::io {

InterfaceGateway::~InterfaceGateway() = default;

//// r = 114. t = 116. rtt = 11400+1160+116 = 12676
//// 1 => Maximal number of connections
InterfaceGateway::InterfaceGateway()
    : webSocketServer(12676, ix::SocketServer::kDefaultHost, ix::SocketServer::kDefaultTcpBacklog, 1), publisher_(InterfacePublisher(webSocketServer)) {
    webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket> webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp());

        webSocketPtr.lock()->setOnMessageCallback([&, webSocketPtr, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
            RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId());
            if (msg->type == ix::WebSocketMessageType::Open) [[unlikely]] {
                RTT_INFO("Sending setup message to client");
                onConnection(webSocketPtr.lock());
                return;
            }

            if (msg->type == ix::WebSocketMessageType::Message) [[likely]] {
                auto envelop = proto::MsgFromInterface{};
                envelop.ParseFromString(msg->str);
                RTT_INFO(envelop.DebugString());
                subscriber_->onMessage(std::move(envelop));
                return;
            }

            RTT_INFO("Received non-message type, ignoring");
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
    game_settings->set_robot_hub_mode(net::robotHubModeToProto(GameSettings::getRobotHubMode()));
    game_settings->set_is_left(GameSettings::isLeft());
    game_settings->set_is_yellow(GameSettings::isYellow());

    const auto ai_settings = setupMessage->mutable_ai_settings();
    ai_settings->set_use_referee(new_interface::RuntimeConfig::useReferee);
    ai_settings->set_ignore_invariants(new_interface::RuntimeConfig::ignoreInvariants);

    std::string state_str = envelope.SerializeAsString();
    wss->sendBinary(state_str);
}

}  // namespace rtt::ai::io