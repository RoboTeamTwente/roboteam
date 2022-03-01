#include <SimulationConfigurationNetworker.hpp>

namespace rtt::net {

SimulationConfigurationPublisher::SimulationConfigurationPublisher() : utils::Publisher(utils::ChannelType::SIMULATION_CONFIGURATION_CHANNEL) {}

bool SimulationConfigurationPublisher::publish(const proto::SimulationConfiguration& config) {
    return this->send(config.SerializeAsString());
}

SimulationConfigurationSubscriber::SimulationConfigurationSubscriber(const std::function<void(const proto::SimulationConfiguration&)>& callback)
    : utils::Subscriber(utils::ChannelType::SIMULATION_CONFIGURATION_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); })
    , callback(callback)
{
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void SimulationConfigurationSubscriber::onPublishedMessage(const std::string& message) {
    proto::SimulationConfiguration simConfig;
    simConfig.ParseFromString(message);
    this->callback(simConfig);
}

}  // namespace rtt::net