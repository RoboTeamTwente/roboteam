#include <WorldNetworker.hpp>

namespace rtt::net {

WorldPublisher::WorldPublisher() : utils::Publisher(utils::ChannelType::WORLD_CHANNEL) {}

std::size_t WorldPublisher::publish(const proto::State& world) {
    return this->send(world.SerializeAsString()) ? world.ByteSizeLong() : -1;
}

WorldSubscriber::WorldSubscriber(const std::function<void(const proto::State&)>& callback)
    : utils::Subscriber(utils::ChannelType::WORLD_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void WorldSubscriber::onPublishedMessage(const std::string& message) {
    proto::State world;
    world.ParseFromString(message);
    this->callback(world);
}

}  // namespace rtt::net