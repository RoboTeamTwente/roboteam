//
// Created by emiel on 21-02-22.
//

#ifndef RTT_WEBSOCKETMANAGER_H
#define RTT_WEBSOCKETMANAGER_H

#include <span>
#include <world/World.hpp>

#include "ixwebsocket/IXWebSocketServer.h"
#include "proto/NewInterface.pb.h"
#include "stp/Play.hpp"

namespace rtt::ai::io {

// Singleton design according to this post on Stackoverflow
// https://stackoverflow.com/questions/1008019/c-singleton-design-pattern

class WebSocketManager {
   public:
    using PlayView = const std::vector<std::unique_ptr<rtt::ai::stp::Play>>&;
    using OnMessageCallback = std::function<void(const proto::InterfaceMessageEnvelope)>;

    static WebSocketManager& instance();

    void broadcastWorld();
    void broadcastSTPStatus(stp::Play* selectedPlay, PlayView plays, int currentTick);

    void broadcastSetupMessageOnNewConnection(PlayView plays);
    void waitForConnection();

    void setOnMessageCallback(OnMessageCallback callback);

    void directDraw(std::basic_string<char> label, proto::Drawing::Color color, proto::Drawing::Method method, std::span<Vector2> points, int retainForTicks);
    void broadcastDrawings();


   public:
    WebSocketManager(WebSocketManager const&) = delete;
    void operator=(WebSocketManager const&) = delete;

   private:
    WebSocketManager();
    void sendMessage(const google::protobuf::Message& message);

    proto::MessageEnvelope drawingBufferEnveloper{};
    bool newClientHasConnect = false;
    ix::WebSocketServer webSocketServer;
};

}







#endif  // RTT_WEBSOCKETMANAGER_H