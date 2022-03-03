#pragma once
#include <proto/SimulationConfiguration.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class SimulationConfigurationPublisher : private utils::Publisher {
   public:
    SimulationConfigurationPublisher();

    // Publishes the given simulation configuration message. Returns success
    bool publish(const proto::SimulationConfiguration& configuration);
};

class SimulationConfigurationSubscriber : private rtt::net::utils::Subscriber {
   public:
    SimulationConfigurationSubscriber(const std::function<void(const proto::SimulationConfiguration&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const proto::SimulationConfiguration&)> callback;
};

}  // namespace rtt::net