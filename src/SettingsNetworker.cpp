#include <SettingsNetworker.hpp>

namespace rtt::net {

SettingsPublisher::SettingsPublisher() : utils::Publisher<proto::Settings>(utils::ChannelType::SETTINGS_CHANNEL) {}

void SettingsPublisher::publish(const proto::Settings& settings) { this->send(settings); }

SettingsSubscriber::SettingsSubscriber(const std::function<void(const proto::Settings&)>& callback)
    : utils::Subscriber<proto::Settings>(utils::ChannelType::SETTINGS_CHANNEL, callback) {}

}  // namespace rtt::net