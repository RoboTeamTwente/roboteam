#include <utils/Publisher.hpp>

namespace rtt::net::utils {

    Publisher::Publisher(const ChannelType channelType)
        : channel(CHANNELS.at(channelType)) {
        this->socket = std::make_unique<zmqpp::socket>(this->context, zmqpp::socket_type::pub);
        this->socket->bind(this->channel.getPublishAddress());
    }

    std::size_t Publisher::send(const std::string& message) {
        zmqpp::message transmission;
        transmission << message;

        return socket->send(transmission, true) ? message.length() : 0;
    }

}  // namespace rtt::net::utils