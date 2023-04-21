#include "utilities/WebSocketManager.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

#include <iostream>

#include "google/protobuf/util/json_util.h"
#include "proto/NewInterface.pb.h"

namespace rtt::ai::io {

WebSocketManager& WebSocketManager::instance() {
    static WebSocketManager instance;
    return instance;
}

void WebSocketManager::waitForConnection() {
    RTT_INFO("Waiting for connection...")
    while (webSocketServer.getClients().empty()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    RTT_INFO("Connection established!")
}

// r = 114. t = 116. rtt = 11400+1160+116 = 12676
WebSocketManager::WebSocketManager() : webSocketServer(12676) {
    webSocketServer.setOnConnectionCallback([onMessageCallback = &onMessageCallback](const std::weak_ptr<ix::WebSocket>& webSocket, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp())

        std::shared_ptr<ix::WebSocket> ws = webSocket.lock();
        if (ws) {
            ws->setOnMessageCallback(
                [onMessageCallback, webSocket, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
                    RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId(), ", ", WebSocketMessageTypeToString(msg->type));

                    if (msg->type == ix::WebSocketMessageType::Message) {
                        proto::InterfaceMessageEnvelope envelop;
                        envelop.ParseFromString(msg->str);

                        if (onMessageCallback->has_value()) {
                            RTT_INFO(envelop.DebugString());
                            onMessageCallback->value()(envelop);
                        }
                    }






//                    if (msg->type == ix::WebSocketMessageType::Message) {
//                        RTT_INFO("Message: ", msg->str);
//                        auto ws = webSocket.lock();
//                        if (ws) {
////                            ws->send("Message received by WebSocketManager", false);
//                        }
//                    }
                }

            );
        }
    });

    std::pair<bool, std::string> res = webSocketServer.listen();
    if (!res.first) {
        RTT_ERROR("Could not start WebSocketServer :", res.second);
    } else {
        webSocketServer.start();
    }
}

std::string WebSocketManager::WebSocketMessageTypeToString(ix::WebSocketMessageType type) {
    switch (type) {
        case ix::WebSocketMessageType::Message:
            return "MESSAGE";
        case ix::WebSocketMessageType::Open:
            return "OPEN";
        case ix::WebSocketMessageType::Close:
            return "CLOSE";
        case ix::WebSocketMessageType::Error:
            return "ERROR";
        case ix::WebSocketMessageType::Ping:
            return "PING";
        case ix::WebSocketMessageType::Pong:
            return "PONG";
        case ix::WebSocketMessageType::Fragment:
            return "FRAGMENT";
        default:
            return "UNKNOWN; fix function pl0x;";
    }
}

void WebSocketManager::sendMessage(const google::protobuf::Message& message, std::basic_string<char> messageType) {
    /* Binary. Around 10 times as fast */
    std::string state_str = message.SerializeAsString();

    // Broadcast to all connected clients
    for (const auto& client : webSocketServer.getClients()) {
        client->sendBinary(state_str);
    }
}

void WebSocketManager::broadcastWorld() {
    static float avg;
    auto started = std::chrono::high_resolution_clock::now();

    proto::MessageEnvelope envelope;
    auto state = io::io.getState();
    envelope.mutable_state()->CopyFrom(state);

    sendMessage(envelope, "world");

    //    auto done = std::chrono::high_resolution_clock::now();
    //    int us = std::chrono::duration_cast<std::chrono::microseconds>(done-started).count();
    //    avg = avg * 0.95 + us*0.05;
    //    std::cout << "broadcast duration: " << avg << " : " << us << std::endl;
}

void WebSocketManager::broadcastPlay(stp::Play* selectedPlay, PlaySpan plays) {
    proto::MessageEnvelope envelope;
    const auto stpStatus = envelope.mutable_stpstatus();
    stpStatus->mutable_selectedplay()->set_playname(selectedPlay->getName());
    stpStatus->mutable_selectedplay()->set_playscore(selectedPlay->lastScore.value_or(-1));


    for (const auto& play : plays) {
        auto scoredPlay = stpStatus->add_scoredplays();
        scoredPlay->set_playname(play->getName());
        scoredPlay->set_playscore(play->getLastScore());
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
        robotMsg.mutable_tactic()->set_name(role->getCurrentTactic()->getName());
        robotMsg.mutable_tactic()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(role->getCurrentTactic()->getStatus())});

        // Skill status
        robotMsg.mutable_skill()->set_name(role->getCurrentTactic()->getCurrentSkill()->getName());
        robotMsg.mutable_skill()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(role->getCurrentTactic()->getCurrentSkill()->getStatus())});
    }

    sendMessage(envelope, "play");
}

void WebSocketManager::broadcastSetupMessage(PlaySpan plays) {
    proto::MessageEnvelope envelope;
    const auto setupMessage = envelope.mutable_setupmessage();

    for (auto& play : plays) {
        setupMessage->add_availableplays(play->getName());
    }

    for (const auto& ruleSet : Constants::ruleSets()) {
        setupMessage->add_availablerulesets(ruleSet.title);
    }

    sendMessage(envelope, "setup");
}

void WebSocketManager::setOnMessageCallback(WebSocketManager::OnMessageCallback callback){
    onMessageCallback = {callback};
}

void WebSocketManager::broadcastDrawings() {

}

}  // namespace rtt::ai::io