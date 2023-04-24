#include "utilities/WebSocketManager.h"

#include <roboteam_utils/Print.h>
#include <utilities/IOManager.h>
#include <ostream>

#include "google/protobuf/util/json_util.h"
#include "stp/PlayDecider.hpp"

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
    webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket>& webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
        RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp());
        newClientHasConnect = true;
        std::shared_ptr<ix::WebSocket> ws = webSocketPtr.lock();
        ws->setOnMessageCallback([connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
            RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId(), ", ", msg->type);
            if (msg->type != ix::WebSocketMessageType::Message) {
                RTT_INFO("Received non-message type, ignoring");
                return;
            }

            auto envelop = proto::InterfaceMessageEnvelope{};
            envelop.ParseFromString(msg->str);
            RTT_INFO(envelop.DebugString());

            switch (envelop.kind_case()) {
                case proto::InterfaceMessageEnvelope::kSetPlay:
                    ai::stp::PlayDecider::lockInterfacePlay(envelop.setplay().playname()); // TODO: How is(was) this thread safe?
                    ai::interface::Output::setRuleSetName(envelop.setplay().rulesetname());
                    break;
                case proto::InterfaceMessageEnvelope::kSetGameSettings: {
                    const auto& gameSettings = envelop.setgamesettings();
                    ai::interface::Output::setUseRefereeCommands(gameSettings.usereferee());
                    SETTINGS.setLeft(gameSettings.isleft());
                    SETTINGS.setYellow(gameSettings.isyellow());
                    SETTINGS.setRobotHubMode(gameSettings.hubmode() == proto::SetGameSettings::BASESTATION ? Settings::RobotHubMode::BASESTATION
                                                                                                           : Settings::RobotHubMode::SIMULATOR);
                    break;
                }
                case proto::InterfaceMessageEnvelope::KindCase::KIND_NOT_SET:
                    RTT_ERROR("Received message with no kind set");
                    break;
            }
        });
    });

    std::pair<bool, std::string> res = webSocketServer.listen();
    if (!res.first) {
        RTT_ERROR("Could not start WebSocketServer :", res.second);
    } else {
        webSocketServer.start();
    }
}

void WebSocketManager::sendMessage(const google::protobuf::Message& message) {
    std::string state_str = message.SerializeAsString();

    // Broadcast to all connected clients
    for (const auto& client : webSocketServer.getClients()) {
        client->sendBinary(state_str);
    }
}

void WebSocketManager::broadcastWorld() {
    proto::MessageEnvelope envelope;

    auto state = io::io.getState();
    envelope.mutable_state()->CopyFrom(state);
    sendMessage(envelope);
}

void WebSocketManager::broadcastSTPStatus(stp::Play* selectedPlay, PlayView plays, int currentTick) {
    auto envelope = proto::MessageEnvelope();
    const auto stpStatus = envelope.mutable_stpstatus();
    stpStatus->set_currenttick(currentTick);
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

    sendMessage(envelope);
}

void WebSocketManager::broadcastSetupMessageOnNewConnection(PlayView plays) {
    if (!newClientHasConnect) {
        return;
    }
    newClientHasConnect = false;

    auto envelope = proto::MessageEnvelope();
    const auto setupMessage = envelope.mutable_setupmessage();
    for (auto& play : plays) {
        setupMessage->add_availableplays(play->getName());
    }

    for (const auto& ruleSet : Constants::ruleSets()) {
        setupMessage->add_availablerulesets(ruleSet.title);
    }

    sendMessage(envelope);
}

void WebSocketManager::directDraw(std::basic_string<char> label, proto::Drawing::Color color, proto::Drawing::Method method, std::span<Vector2> points, int retainForTicks = 1) {
    const auto drawing = drawingBufferEnveloper.mutable_drawingbuffer()->add_buffer();
    drawing->set_label(label);
    drawing->set_color(color);
    drawing->set_method(method);
    drawing->set_retainforticks(retainForTicks);
    for (auto& point : points) {
        auto protoPoint = drawing->add_points();
        protoPoint->set_x(point.x);
        protoPoint->set_y(point.y);
    }
}

void WebSocketManager::broadcastDrawings() {
    sendMessage(drawingBufferEnveloper);
    drawingBufferEnveloper.mutable_drawingbuffer()->clear_buffer();
}

}  // namespace rtt::ai::io

namespace ix {
inline std::ostream& operator<<(std::ostream& str, const ix::WebSocketMessageType& type) {
    return str << ([&type]() {
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
           })();
}
}  // namespace ix