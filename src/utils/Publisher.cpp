#include <utils/Publisher.hpp>

namespace rtt::net::utils {

Publisher::Publisher(const ChannelType &channelType) : channel(CHANNELS.at(channelType)) {
    this->socket = std::make_unique<zmqpp::socket>(this->context, zmqpp::socket_type::pub);
    this->socket->bind(this->channel.getPublishAddress());
}

bool Publisher::send(const std::string &message) {
    zmqpp::message transmission;
    transmission << message;
    return socket->send(transmission, true);
}

} // namespace rtt::net::utils