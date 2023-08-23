//
// Created by Martin Miksik on 25/04/2023.
//

#include "gui/networking/InterfaceGateway.h"

#include <memory>

#include "STPManager.h"
#include "gui/networking/InterfaceSubscriber.h"

namespace rtt::ai::gui::net {

InterfaceGateway::~InterfaceGateway() = default;

InterfacePublisher& InterfaceGateway::publisher() { return _publisher; }

//// r = 114. t = 116. rtt = 11400+1160+116 = 12676

InterfaceGateway::InterfaceGateway(int port) : webSocketServer(port), _publisher(InterfacePublisher(webSocketServer)), _subscriber(std::make_unique<InterfaceSubscriber>()) {
    webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket> webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp(), " marked with id: ", connectionState->getId());

        webSocketPtr.lock()->setOnMessageCallback([&, webSocketPtr, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
            switch (msg->type) {
                case ix::WebSocketMessageType::Open: {
                    RTT_INFO("Sending setup message to client [ip: ", connectionState->getRemoteIp(), ", id: ", connectionState->getId(), "]");
                    connectionState->getId();
                    _publisher.publishAIStatus();
                } break;
                case ix::WebSocketMessageType::Message: {
                    RTT_INFO("Received message from client [ip: ", connectionState->getRemoteIp(), ", id: ", connectionState->getId(), "]");
                    auto envelop = proto::MsgFromInterface{};
                    envelop.ParseFromString(msg->str);
                    _subscriber->onMessage(std::move(envelop));

                    // Republish AI status to update all clients in case the AI has changed something
                    _publisher.publishAIStatus();
                } break;
                default: {
                    RTT_WARNING("Received non-message type, ignoring [ip: ", connectionState->getRemoteIp(), ", id: ", connectionState->getId(), "]");
                } break;
            }
        });
    });

    const auto [didSucceed, errorLog] = webSocketServer.listen();
    if (!didSucceed) {
        RTT_ERROR("Could not start WebSocketServer :", errorLog);
        return;
    }

    RTT_SUCCESS("WebSocketServer started on port ", port);
    webSocketServer.start();
}

}  // namespace rtt::ai::gui::net