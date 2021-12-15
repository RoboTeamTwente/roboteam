#pragma once
#include <proto/State.pb.h>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include <functional>

namespace rtt::net {

class WorldPublisher : private utils::Publisher<proto::State> {
   public:
    WorldPublisher();

    bool publish(const proto::State& world);
};

class WorldSubscriber : private rtt::net::utils::Subscriber<proto::State> {
   public:
    WorldSubscriber(const std::function<void(const proto::State&)>& callback);
};

}  // namespace rtt::net