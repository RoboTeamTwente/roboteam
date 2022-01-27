#pragma once
#include <proto/SimulationConfiguration.pb.h>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include <functional>

namespace rtt::net {

class SimulationConfigurationPublisher : private utils::Publisher<proto::SimulationConfiguration> {
   public:
    SimulationConfigurationPublisher();

    bool publish(const proto::SimulationConfiguration& configuration);
};

class SimulationConfigurationSubscriber : private rtt::net::utils::Subscriber<proto::SimulationConfiguration> {
   public:
    SimulationConfigurationSubscriber(const std::function<void(const proto::SimulationConfiguration&)>& configuration);
};

}  // namespace rtt::net