//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACEGATEWAY_H
#define RTT_INTERFACEGATEWAY_H

#include <span>
#include "ixwebsocket/IXWebSocketServer.h"

#include "Interface.h"
#include "proto/NewInterface.pb.h"
#include "stp/Play.hpp"

namespace rtt::ai::io {

class InterfaceGateway {
   public:
    using PlayView = const std::vector<std::unique_ptr<rtt::ai::stp::Play>>&;
    InterfaceGateway(InterfaceGateway const&) = delete;
    void operator=(InterfaceGateway const&) = delete;

    static InterfaceGateway& instance();

    void broadcastSTPStatus(stp::Play* selectedPlay, PlayView plays, int currentTick);
    void broadcastWorld();
    void broadcastVisuals();


   private:
    ix::WebSocketServer webSocketServer;

    InterfaceGateway();
    void onConnection(const std::shared_ptr<ix::WebSocket>& wss);
    void onMessage(const proto::MsgFromInterface&& message);
    void broadcastMessage(const google::protobuf::Message& message);
};

}  // namespace rtt::ai::new_interface

#endif  // RTT_INTERFACEGATEWAY_H
