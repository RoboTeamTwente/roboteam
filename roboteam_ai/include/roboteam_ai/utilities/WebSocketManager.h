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
    using PlaySpan = std::span<std::unique_ptr<rtt::ai::stp::Play>>;
    using OnMessageCallback = std::function<void(const proto::InterfaceMessageEnvelope)>;

    static WebSocketManager& instance();

    void broadcastWorld();
    void broadcastPlay(stp::Play* selectedPlay, PlaySpan plays);
    void broadcastDrawings();
    void broadcastSetupMessage(PlaySpan plays);
    void waitForConnection();

    void setOnMessageCallback(OnMessageCallback callback);


   public:
    WebSocketManager(WebSocketManager const&) = delete;
    void operator=(WebSocketManager const&) = delete;

   private:
    WebSocketManager();
    void sendMessage(const google::protobuf::Message& message, std::basic_string<char> messageType);
    std::optional<OnMessageCallback> onMessageCallback;

    static std::string WebSocketMessageTypeToString(ix::WebSocketMessageType type);
    ix::WebSocketServer webSocketServer;
};

}







#endif  // RTT_WEBSOCKETMANAGER_H