#ifndef RTT_INTERFACESUBSCRIBER_H
#define RTT_INTERFACESUBSCRIBER_H
#include <atomic>

#include "proto/GUI.pb.h"

namespace rtt::ai::gui::net {
class InterfaceGateway;
class InterfaceSubscriber {
   public:
    /**
     * @brief Callback function invoked when a message is received from a client.
     * Processes the received message and performs corresponding actions.
     *
     * @param message The received message from the client.
     */
    void onMessage(const proto::MsgFromInterface&& message);
    InterfaceSubscriber() = default;
};
}  // namespace rtt::ai::gui::net
#endif  // RTT_INTERFACESUBSCRIBER_H
