#pragma once

#include <utils/Channel.hpp>
#include <utils/Channels.hpp>

#include <zmqpp/zmqpp.hpp>
#include <string>
#include <memory>

namespace rtt::net::utils {

/* Defines a publisher that publishes protobuf messages to a TCP channel using zmqpp.
   Only one publisher per channel per computer can exist at the same time. */
template<class Message>
class Publisher {
public:
    /* Apply rule of 5: in all cases we cannot copy/move this object */
    Publisher(const Publisher &copy) = delete;
    Publisher &operator=(const Publisher &other) = delete;
    Publisher(const Publisher &&copy) = delete;
    Publisher &operator=(Publisher &&data) = delete;

    /* Open a publisher and bind to the specified channel.
     * @param channel: The channel to publish to. */
    explicit Publisher(const ChannelType &channelType) : channel(CHANNELS.at(channelType)) {
        this->socket = std::make_unique<zmqpp::socket>(this->context, zmqpp::socket_type::pub);
        this->socket->bind(this->channel.getPublishAddress());
    }

protected:
    /* Send a message of the specified type
     * @param message: Message to send. */
    bool send(const Message &message) {
        std::string serializedMessage = message.SerializeAsString();
        
        zmqpp::message transmission;
        transmission << serializedMessage;
        
        return socket->send(transmission, true);
    }

private:
    zmqpp::context context;
    std::unique_ptr<zmqpp::socket> socket;
    Channel channel;

};
}  // namespace rtt::net::utils
