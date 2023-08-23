//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACEGATEWAY_H
#define RTT_INTERFACEGATEWAY_H

#include <memory>
#include <span>

#include "InterfacePublisher.h"
#include "ixwebsocket/IXWebSocketServer.h"
#include "proto/GUI.pb.h"
#include "stp/Play.hpp"

namespace rtt::ai::gui::net {
class InterfaceSubscriber;

/**
 * @class InterfaceGateway
 * @brief Singleton class that handles the communication between the Interface and the rest of the AI.
 *
 * The communication is split into two parts:
 * -> out: InterfacePublisher takes care of sending messages to the Interface.
 * -> in:  InterfaceSubscriber takes care of receiving messages from the Interface.
 *
 * While both parts are relatively small and could be part of the InterfaceGateway, they are separated
 * to make the data flow (hopefully) more clear.
 */
class InterfaceGateway {
   private:
    ix::WebSocketServer webSocketServer;

    /// Publisher is not forward declared, because it is accessed from outside the InterfaceGateway, so we need to know the available methods;
    InterfacePublisher _publisher;

    /// Subscriber is forward declared, so that changes to the subscriber do not require recompilation of the InterfaceGateway
    /// !This is purely for convenience!
    /// On that account it has to be a unique_ptr, because the compiler does not know the size of the object
    std::unique_ptr<InterfaceSubscriber> _subscriber;

   public:
    explicit InterfaceGateway(int port);
    ~InterfaceGateway();
    InterfaceGateway(InterfaceGateway const&) = delete;
    void operator=(InterfaceGateway const&) = delete;

    /**
     * @brief Reference to the InterfaceGateway instance.
     * @return InterfaceGateway instance.
     */
    InterfacePublisher& publisher();
};

}  // namespace rtt::ai::gui::net

#endif  // RTT_INTERFACEGATEWAY_H
