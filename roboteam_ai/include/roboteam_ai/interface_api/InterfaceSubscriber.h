//
// Created by Martin Miksik on 24/05/2023.
//

#ifndef RTT_INTERFACESUBSCRIBER_H
#define RTT_INTERFACESUBSCRIBER_H
#include <atomic>
#include "proto/NewInterface.pb.h"
#include <QtNetwork>

namespace rtt::ai::io {
class InterfaceGateway;
class InterfaceSubscriber {
    InterfaceSubscriber() = default;

   public:
    /**
     * @brief Callback function invoked when a message is received from a client.
     * Processes the received message and performs corresponding actions.
     *
     * @param message The received message from the client.
     */
    void onMessage(const proto::MsgFromInterface&& message);

   private:
    void sendPacketToSimulator(const SimulatorCommand& packet);

    QUdpSocket simulator_socket;
};
}  // namespace rtt::ai::io
#endif  // RTT_INTERFACESUBSCRIBER_H
