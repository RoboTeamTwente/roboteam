#include <utils/Channel.hpp>

namespace rtt::net::utils {

Channel::Channel(std::string name, std::string ip, std::string port) : name(std::move(name)), ip(std::move(ip)), port(std::move(port)) {}

Channel::Channel(const Channel &other) : name(other.name), ip(other.ip), port(other.port) {}

bool Channel::operator==(const Channel &other) { return name == other.name && port == other.port; }

bool Channel::operator!=(const Channel &other) { return !(*this == other); }

std::string Channel::getSubscribeAddress() { return getAddress(ip, port); }

std::string Channel::getPublishAddress() { return getAddress("*", port); }

std::string Channel::getAddress(const std::string &_ip, const std::string &_port) { return "tcp://" + _ip + ":" + _port; }

std::string Channel::toInfoString(bool isPublisher) {
    if (isPublisher) {
        return "\"" + name + "\"" + " at address: " + getPublishAddress();
    }
    return "\"" + name + "\"" + " at address: " + getSubscribeAddress();
}

} // namespace rtt::net::utils