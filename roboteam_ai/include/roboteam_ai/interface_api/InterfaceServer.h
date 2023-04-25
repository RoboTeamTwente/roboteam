//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACESERVER_H
#define RTT_INTERFACESERVER_H

#include <span>
#include "ixwebsocket/IXWebSocketServer.h"

#include "Interface.h"
#include "stp/Play.hpp"

namespace rtt::ai::io {

class InterfaceServer {
   public:
    using PlayView = const std::vector<std::unique_ptr<rtt::ai::stp::Play>>&;
    InterfaceServer(InterfaceServer const&) = delete;
    void operator=(InterfaceServer const&) = delete;

    static InterfaceServer& instance();

    void broadcastSTPStatus(stp::Play* selectedPlay, PlayView plays, int currentTick);
    void broadcastWorld();
    void broadcastVisuals();


   private:
    ix::WebSocketServer webSocketServer;

    InterfaceServer();
    void onConnection(const std::shared_ptr<ix::WebSocket>& wss);
    void onMessage(const proto::InterfaceMessageEnvelope&& message);
    void broadcastMessage(const google::protobuf::Message& message);
};

}  // namespace rtt::ai::new_interface

#endif  // RTT_INTERFACESERVER_H
