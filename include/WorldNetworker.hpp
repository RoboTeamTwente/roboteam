#include <proto/World.pb.h>

#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

#include <functional>

namespace rtt::net {

class WorldPublisher : private utils::Publisher<proto::World> {
public:
    WorldPublisher();

    void publish(const proto::World& world);
};

class WorldSubscriber : private utils::Subscriber<proto::World> {
public:
    WorldSubscriber(const std::function<void(const proto::World&)>& callback);

};

} // namespace rtt::net