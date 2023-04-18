#include "utilities/WebSocketManager.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>

#include <iostream>

#include "google/protobuf/util/json_util.h"
#include "proto/PlayData.pb.h"

namespace rtt::ai::io {

WebSocketManager& WebSocketManager::instance() {
    static WebSocketManager instance;
    return instance;
}

// r = 114. t = 116. rtt = 11400+1160+116 = 12676
WebSocketManager::WebSocketManager() : webSocketServer(12676){

    webSocketServer.setOnConnectionCallback(
        [this](const std::weak_ptr<ix::WebSocket>& webSocket, const std::shared_ptr<ix::ConnectionState>& connectionState){
            RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp())

            std::shared_ptr<ix::WebSocket> ws = webSocket.lock();
            if(ws){
                ws->setOnMessageCallback(

                    [webSocket, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg){
                        RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId(), ", ", WebSocketMessageTypeToString(msg->type));
                        if(msg->type == ix::WebSocketMessageType::Message){
                            RTT_INFO("Message: ", msg->str);
                            auto ws = webSocket.lock();
                            if(ws){
                                ws->send("Message received by WebSocketManager", false);
                            }
                        }
                    }

                );
            }
        }
    );

    std::pair<bool, std::string> res = webSocketServer.listen();
    if(!res.first){
        RTT_ERROR("Could not start WebSocketServer :", res.second);
    }else{
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

void WebSocketManager::broadcastWorld(){
    static float avg;

    auto started = std::chrono::high_resolution_clock::now();

    auto state = io::io.getState();
    /* JSON */
    std::string state_str;
    google::protobuf::util::MessageToJsonString(state, &state_str);
    std::string json = R"({"type":"world","data":)" + state_str + "}";

    /* Binary. Around 10 times as fast */
//        std::string state_str = state.SerializeAsString();

    // Broadcast to all connected clients
    for(const auto& client : webSocketServer.getClients()){
        client->send(json, false);
    }

    auto done = std::chrono::high_resolution_clock::now();
    int us = std::chrono::duration_cast<std::chrono::microseconds>(done-started).count();
    avg = avg * 0.95 + us*0.05;
    std::cout << "broadcast duration: " << avg << " : " << us << std::endl;
}

void WebSocketManager::broadcastPlay(stp::Play* play) {
    proto::PlayData playData;
    playData.set_name(play->getName());
    playData.set_score(play->lastScore.value_or(-1));

    for (auto& [role, status] : play->getRoleStatuses()) {
        auto robotMsg = playData.add_robots();
        robotMsg->set_id(role->getCurrentRobot().value()->getId());

        // Role status
        robotMsg->mutable_role()->set_name(role->getName());
        robotMsg->mutable_role()->set_status(proto::Robot::Status{static_cast<int>(status)});

        // Tactic status
        robotMsg->mutable_tactic()->set_name(role->getCurrentTactic()->getName());
        robotMsg->mutable_tactic()->set_status(proto::Robot::Status{static_cast<int>(role->getCurrentTactic()->getStatus())});

        // Skill status
//        auto skillMsg = proto::Robot::Skill {};
        robotMsg->mutable_skill()->set_name(role->getCurrentTactic()->getCurrentSkill()->getName());
        robotMsg->mutable_skill()->set_status(proto::Robot::Status{static_cast<int>(role->getCurrentTactic()->getCurrentSkill()->getStatus())});
    }

    std::string state_str;
    google::protobuf::util::JsonPrintOptions options;
    options.always_print_enums_as_ints = true;
    options.always_print_primitive_fields = true;

    google::protobuf::util::MessageToJsonString(playData, &state_str, options);
    std::string json = R"({"type":"play","data":)" + state_str + "}";

    for(const auto& client : webSocketServer.getClients()){
        client->send(json, false);
    }
}

}