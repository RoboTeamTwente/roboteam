//#include "ixwebsocket/IXWebSocketServer.h"
//#include "stp/Play.hpp"
//#include "stp/PlayDecider.hpp"
//#include "utilities/IOManager.h"
//
//#include "STPManager.h"
//#include <span>
//
//namespace rtt::ai::new_interface {
//
//class Interface {
//   public:
//    static auto draw(std::basic_string<char> label, proto::Drawing::Color color, proto::Drawing::Method method, std::span<Vector2> points, int retainForTicks = 1) -> void {
//        const double orientation = SETTINGS.isLeft() ? -1 : 1;
//        const auto drawing = drawingsBuffer.mutable_drawingbuffer()->add_buffer();
//        drawing->set_label(label);
//        drawing->set_color(color);
//        drawing->set_method(method);
//        drawing->set_retainforticks(retainForTicks);
//        for (auto& point : points) {
//            auto protoPoint = drawing->add_points();
//            protoPoint->set_x(orientation * point.x);
//            protoPoint->set_y(orientation * point.y);
//        }
//    }
//
//    static auto consumeBuffers(std::function<void(proto::MessageEnvelope&, proto::MessageEnvelope&)> consumer) {
//        consumer(drawingsBuffer, metricsBuffer);
//        drawingsBuffer.mutable_drawingbuffer()->clear_buffer();
//        metricsBuffer.mutable_metricbuffer()->clear_buffer();
//    }
//
//   private:
//    static proto::MessageEnvelope drawingsBuffer;
//    static proto::MessageEnvelope metricsBuffer;
//};
//
//
//class InterfaceServer {
//   public:
//    using PlayView = const std::vector<std::unique_ptr<rtt::ai::stp::Play>>&;
//
//    static auto instance() -> InterfaceServer& {
//        static InterfaceServer instance;
//        return instance;
//    }
//
//    auto sendSTPStatus(stp::Play* selectedPlay, PlayView plays, int currentTick) -> void {
//        auto envelope = proto::MessageEnvelope();
//        const auto stpStatus = envelope.mutable_stpstatus();
//        stpStatus->set_currenttick(currentTick);
//        stpStatus->mutable_selectedplay()->set_playname(selectedPlay->getName());
//        stpStatus->mutable_selectedplay()->set_playscore(selectedPlay->lastScore.value_or(-1));
//
//        for (const auto& play : plays) {
//            auto scoredPlay = stpStatus->add_scoredplays();
//            scoredPlay->set_playname(play->getName());
//            scoredPlay->set_playscore(play->getLastScore());
//        }
//
//        auto robotsMap = stpStatus->mutable_robots();
//        for (auto& [role, status] : selectedPlay->getRoleStatuses()) {
//            int robotId = role->getCurrentRobot()->get()->getId();
//            (*robotsMap)[robotId] = proto::STPStatus::STPRobot();
//            auto& robotMsg = robotsMap->at(robotId);
//            robotMsg.set_id(role->getCurrentRobot().value()->getId());
//
//            // Role status
//            robotMsg.mutable_role()->set_name(role->getName());
//            robotMsg.mutable_role()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(status)});
//
//            // Tactic status
//            robotMsg.mutable_tactic()->set_name(role->getCurrentTactic()->getName());
//            robotMsg.mutable_tactic()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(role->getCurrentTactic()->getStatus())});
//
//            // Skill status
//            robotMsg.mutable_skill()->set_name(role->getCurrentTactic()->getCurrentSkill()->getName());
//            robotMsg.mutable_skill()->set_status(proto::STPStatus::STPRobot::Status{static_cast<int>(role->getCurrentTactic()->getCurrentSkill()->getStatus())});
//        }
//        broadcastMessage(envelope);
//    }
//
//    auto sendWorld() -> void {
//        auto envelope = proto::MessageEnvelope();
//        auto state = io::io.getState();
//        envelope.mutable_state()->CopyFrom(state);
//        broadcastMessage(envelope);
//    }
//
//    auto sendVisuals() -> void {
//        Interface::consumeBuffers([&](auto& drawings, auto& metrics) {
//            broadcastMessage(drawings);
//            broadcastMessage(metrics);
//        });
//    }
//
//   private:
//    ix::WebSocketServer webSocketServer;
//    InterfaceServer() : webSocketServer(12676) {
//        webSocketServer.setOnConnectionCallback([&](const std::weak_ptr<ix::WebSocket>& webSocketPtr, const std::shared_ptr<ix::ConnectionState>& connectionState) {
//            RTT_INFO("WebSocket connection incoming from ", connectionState->getRemoteIp());
//            std::shared_ptr<ix::WebSocket> ws = webSocketPtr.lock();
//            onConnection(ws);
//
//            ws->setOnMessageCallback([&, connectionState](const std::unique_ptr<ix::WebSocketMessage>& msg) {
//                RTT_INFO(connectionState->getRemoteIp(), ", ", connectionState->getId());
//                if (msg->type != ix::WebSocketMessageType::Message) {
//                    RTT_INFO("Received non-message type, ignoring");
//                    return;
//                }
//
//                auto envelop = proto::InterfaceMessageEnvelope{};
//                envelop.ParseFromString(msg->str);
//                RTT_INFO(envelop.DebugString());
//                onMessage(std::move(envelop));
//            });
//        });
//    }
//
//    void onConnection(std::shared_ptr<ix::WebSocket> wss) {
//        auto envelope = proto::MessageEnvelope();
//        const auto setupMessage = envelope.mutable_setupmessage();
//        for (auto& play : rtt::STPManager::plays) { // TODO: How is(was) this thread safe?
//            setupMessage->add_availableplays(play->getName());
//        }
//
//        for (const auto& ruleSet : Constants::ruleSets()) {
//            setupMessage->add_availablerulesets(ruleSet.title);
//        }
//
//        std::string state_str = envelope.SerializeAsString();
//        wss->sendBinary(state_str);
//    }
//
//    void onMessage(const proto::InterfaceMessageEnvelope&& message) {
//        RTT_INFO(message.DebugString());
//
//        switch (message.kind_case()) {
//            case proto::InterfaceMessageEnvelope::kSetPlay:
//                ai::stp::PlayDecider::lockInterfacePlay(message.setplay().playname()); // TODO: How is(was) this thread safe?
//                ai::interface::Output::setRuleSetName(message.setplay().rulesetname());
//                break;
//            case proto::InterfaceMessageEnvelope::kSetGameSettings: {
//                const auto& gameSettings = message.setgamesettings();
//                ai::interface::Output::setUseRefereeCommands(gameSettings.usereferee());
//                SETTINGS.setLeft(gameSettings.isleft());
//                SETTINGS.setYellow(gameSettings.isyellow());
//                SETTINGS.setRobotHubMode(gameSettings.hubmode() == proto::SetGameSettings::BASESTATION ? Settings::RobotHubMode::BASESTATION
//                                                                                                       : Settings::RobotHubMode::SIMULATOR);
//                break;
//            }
//            case proto::InterfaceMessageEnvelope::KindCase::KIND_NOT_SET:
//                RTT_ERROR("Received message with no kind set");
//                break;
//        }
//    }
//
//    void broadcastMessage(const google::protobuf::Message& message) {
//        std::string state_str = message.SerializeAsString();
//        for (const auto& client : webSocketServer.getClients()) {
//            client->sendBinary(state_str);
//        }
//    }
//};
//
//}