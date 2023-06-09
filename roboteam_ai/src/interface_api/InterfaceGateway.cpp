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

InterfaceGateway::InterfaceGateway(int port)
    : webSocketServer(port), publisher_(InterfacePublisher(webSocketServer)) {
    webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket> webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp());
        webSocketPtr.lock()->setOnMessageCallback([&, webSocketPtr, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
            RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId());
            if (msg->type == ix::WebSocketMessageType::Open) [[unlikely]] {
                RTT_INFO("Sending setup message to client");
                connectionState->getId();
                publisher_.publishAIStatus();
                return;
            }

            if (msg->type == ix::WebSocketMessageType::Message) [[likely]] {
                auto envelop = proto::MsgFromInterface{};
                envelop.ParseFromString(msg->str);
                RTT_INFO(envelop.DebugString());
                subscriber_->onMessage(std::move(envelop));

                // Republish AI status to update all clients in case the AI has changed something
                publisher_.publishAIStatus();
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
}  // namespace rtt::ai::io