#include <SettingsNetworker.hpp>

namespace rtt::net {

SettingsPublisher::SettingsPublisher() : utils::Publisher<proto::Setting>(utils::ChannelType::SETTINGS_CHANNEL) {}

bool SettingsPublisher::publish(const proto::Setting& settings) { return this->send(settings); }

SettingsSubscriber::SettingsSubscriber(const std::function<void(const proto::Setting&)>& callback)
    : utils::Subscriber<proto::Setting>(utils::ChannelType::SETTINGS_CHANNEL, callback) {}

}  // namespace rtt::net