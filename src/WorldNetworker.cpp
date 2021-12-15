#include <WorldNetworker.hpp>

namespace rtt::net {

WorldPublisher::WorldPublisher() : utils::Publisher<proto::State>(utils::ChannelType::WORLD_CHANNEL) {}

bool WorldPublisher::publish(const proto::State& world) { return this->send(world); }

WorldSubscriber::WorldSubscriber(const std::function<void(const proto::State&)>& callback) : utils::Subscriber<proto::State>(utils::ChannelType::WORLD_CHANNEL, callback) {}

}  // namespace rtt::net