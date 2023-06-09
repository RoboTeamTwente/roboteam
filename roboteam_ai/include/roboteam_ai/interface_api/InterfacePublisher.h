//
// Created by Martin Miksik on 24/05/2023.
//

#ifndef RTT_INTERFACEPUBLISHER_H
#define RTT_INTERFACEPUBLISHER_H

#include "stp/Play.hpp"

#include <proto/NewInterface.pb.h>
#include <ixwebsocket/IXWebSocketServer.h>

#include <functional>

namespace rtt::ai::io {
class InterfaceGateway;

class InterfacePublisher {
    friend InterfaceGateway;  /// Only the InterfaceGateway can create an InterfacePublisher

   public:
    using PlayView = const std::vector<std::unique_ptr<rtt::ai::stp::Play>>&;
    /**
     * @brief Broadcasts the STP status information to connected clients.
     * @param selectedPlay Pointer to the selected Play object.
     * @param plays Vector of unique pointers to Play objects.
     * @param currentTick Current game tick.
     */
    InterfacePublisher& publishStpStatus(stp::Play* selectedPlay, PlayView plays, int currentTick);

    /**
     * @brief Broadcasts the current game state to connected clients.
     */
    InterfacePublisher& publishWorld();

    /**
     * @brief Broadcasts visualizations to connected clients.
     * Retrieves visualizations from the Interface module and sends them in a message.
     */
    InterfacePublisher& publishVisuals();

    InterfacePublisher& publishAIStatus();

   private:
    ix::WebSocketServer& wss;
    explicit InterfacePublisher(ix::WebSocketServer& wss);

    /**
     * @brief Sends a message to all connected clients.
     * @param message The message to be sent.
     */
    void publishProtoMessage(const google::protobuf::Message& message) {
        std::string state_str = message.SerializeAsString();
        for (auto& client : wss.getClients()) {
            client->sendBinary(state_str);
        }
    }
};
}  // namespace rtt::ai::io
#endif  // RTT_INTERFACEPUBLISHER_H
