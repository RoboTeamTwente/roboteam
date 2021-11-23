#pragma once

#include <utils/Channel.hpp>
#include <utils/Channels.hpp>

#include <zmqpp/zmqpp.hpp>
#include <string>
#include <memory>

namespace rtt::net::utils {

// Defines a publisher that publishes to a TCP channel
class Publisher {
public:
    // Apply rule of 5: in all cases we cannot copy/move this object
    Publisher(const Publisher &copy) = delete;
    Publisher &operator=(const Publisher &other) = delete;
    Publisher(const Publisher &&copy) = delete;
    Publisher &operator=(Publisher &&data) = delete;

    /*
     * Open a publisher and bind to the channel.
     * @param channel: The channel to publish to.
     */
    explicit Publisher(const ChannelType &channelType);

protected:
    /*
     * Send a message of the specified type
     * @param message: message to send. Must consist of base google::protobuf::Message to compile!
     */
    bool send(const std::string &message);

private:
    zmqpp::context context;
    std::unique_ptr<zmqpp::socket> socket;
    Channel channel;

};
}  // namespace rtt::net::utils
