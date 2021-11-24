#include <WorldNetworker.hpp>

namespace rtt::net {

WorldPublisher::WorldPublisher() : utils::Publisher<proto::World>(utils::ChannelType::SETTINGS_PRIMARY_CHANNEL) {}

void WorldPublisher::publish(const proto::World& world) { this->send(world); }

WorldSubscriber::WorldSubscriber(const std::function<void(const proto::World&)>& callback)
    : utils::Subscriber<proto::World>(utils::ChannelType::SETTINGS_PRIMARY_CHANNEL, callback) {}

}  // namespace rtt::net