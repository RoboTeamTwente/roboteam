#include <proto/Settings.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class SettingsPublisher : private utils::Publisher<proto::Settings> {
   public:
    SettingsPublisher();

    void publish(const proto::Settings& settings);
};

class SettingsSubscriber : private utils::Subscriber<proto::Settings> {
   public:
    SettingsSubscriber(const std::function<void(const proto::Settings&)>& callback);
};

}  // namespace rtt::net