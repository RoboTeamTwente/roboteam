#include <SimulationConfigurationNetworker.hpp>

namespace rtt::net {

SimulationConfigurationPublisher::SimulationConfigurationPublisher() : utils::Publisher<proto::SimulationConfiguration>(utils::ChannelType::SIMULATION_CONFIGURATION_CHANNEL) {}

bool SimulationConfigurationPublisher::publish(const proto::SimulationConfiguration& config) { return this->send(config); }

SimulationConfigurationSubscriber::SimulationConfigurationSubscriber(const std::function<void(const proto::SimulationConfiguration&)>& callback) : utils::Subscriber<proto::SimulationConfiguration>(utils::ChannelType::SIMULATION_CONFIGURATION_CHANNEL, callback) {}

}  // namespace rtt::net