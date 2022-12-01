#pragma once

#include <memory>
#include <string>
#include <utils/Channel.hpp>
#include <utils/Channels.hpp>
#include <zmqpp/zmqpp.hpp>

namespace rtt::net::utils {

/* Defines a publisher that publishes protobuf messages to a TCP channel using zmqpp.
   Only one publisher per channel per computer can exist at the same time. */
class Publisher {
   public:
    /* Apply rule of 5: in all cases we cannot copy/move this object */
    Publisher(const Publisher &copy) = delete;
    Publisher &operator=(const Publisher &other) = delete;
    Publisher(const Publisher &&copy) = delete;
    Publisher &operator=(Publisher &&data) = delete;

    /* Open a publisher and bind to the specified channel.
     * @param channel: The channel to publish to. */
    explicit Publisher(const ChannelType channelType);

   protected:
    /* Send a message of the specified type
     * @param message: Message to send.
     * @returns bytes sent, or 0 on failure */
    std::size_t send(const std::string &message);

   private:
    zmqpp::context context;
    std::unique_ptr<zmqpp::socket> socket;
    Channel channel;
};
}  // namespace rtt::net::utils
