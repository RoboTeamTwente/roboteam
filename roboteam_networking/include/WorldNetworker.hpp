#pragma once
#include <proto/State.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

    class WorldPublisher : private utils::Publisher {
    public:
        WorldPublisher();

        // Publishes the given world. Returns bytes sent, 0 on failure
        std::size_t publish(const proto::State& world);
    };

    class WorldSubscriber : private rtt::net::utils::Subscriber {
    public:
        WorldSubscriber(const std::function<void(const proto::State&)>& callback);

    private:
        void onPublishedMessage(const std::string& message);
        const std::function<void(const proto::State&)> callback;
    };

}  // namespace rtt::net