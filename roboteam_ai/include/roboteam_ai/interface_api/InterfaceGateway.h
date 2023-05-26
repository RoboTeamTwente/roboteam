//
// Created by Martin Miksik on 25/04/2023.
//

#ifndef RTT_INTERFACEGATEWAY_H
#define RTT_INTERFACEGATEWAY_H

#include <memory>
#include <span>

#include "InterfacePublisher.h"
#include "Out.h"
#include "ixwebsocket/IXWebSocketServer.h"
#include "proto/NewInterface.pb.h"
#include "stp/Play.hpp"

namespace rtt::ai::io {
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
    InterfacePublisher publisher_;

    /// Subscriber is forward declared, so that changes to the subscriber do not require recompilation of the InterfaceGateway
    /// On that account it has to be a unique_ptr, because the compiler does not know the size of the object
    std::unique_ptr<InterfaceSubscriber> subscriber_;

    InterfaceGateway();

   public:
    ~InterfaceGateway();
    InterfaceGateway(InterfaceGateway const&) = delete;
    void operator=(InterfaceGateway const&) = delete;

    /**
     * @brief Retrieves the singleton instance of the InterfacePublisher.
     * It also initializes the InterfaceGateway singleton if it has not been initialized yet.
     * @return Reference to the InterfaceGateway instance.
     */
    inline static InterfacePublisher& publisher() {
        static InterfaceGateway instance;
        return instance.publisher_;
    };

    /**
     * @brief Initializes the InterfaceGateway singleton, but do not provide a reference to it!
     */
    inline static void init() { publisher(); }
};

}  // namespace rtt::ai::io

#endif  // RTT_INTERFACEGATEWAY_H
