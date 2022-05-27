#include <SettingsNetworker.hpp>

namespace rtt::net {

SettingsPublisher::SettingsPublisher() : utils::Publisher(utils::ChannelType::SETTINGS_CHANNEL) {}

std::size_t SettingsPublisher::publish(const proto::Setting& settings) {
    return this->send(settings.SerializeAsString()) ? settings.ByteSizeLong() : -1;
}

SettingsSubscriber::SettingsSubscriber(const std::function<void(const proto::Setting&)>& callback)
    : utils::Subscriber(utils::ChannelType::SETTINGS_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void SettingsSubscriber::onPublishedMessage(const std::string& message) {
    proto::Setting settings;
    settings.ParseFromString(message);
    this->callback(settings);
}

}  // namespace rtt::net