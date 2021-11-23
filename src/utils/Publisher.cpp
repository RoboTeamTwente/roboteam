#include <utils/Publisher.hpp>

namespace rtt::net::utils {

Publisher::Publisher(const ChannelType &channelType) : channel(CHANNELS.at(channelType)) {
    std::cout << "Starting roboteam_proto publisher for channel " << channel.toInfoString(true) << std::endl;
    socket = new zmqpp::socket(context, zmqpp::socket_type::pub);
    socket->bind(channel.getPublishAddress());
}

Publiser::~Publisher() {
    std::cout << "Stopping roboteam_proto publisher for channel " << channel.toInfoString(true) << std::endl;
    socket->close();
    delete socket;
}

bool Publisher::send(const T &message) {
    zmqpp::message transmission;
    transmission << message.SerializeAsString();
    return socket->send(transmission, true);
}

} // namespace rtt::net::utils