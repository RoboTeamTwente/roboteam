#pragma once
#include <proto/Setting.pb.h>

#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class SettingsPublisher : private utils::Publisher<proto::Setting> {
   public:
    SettingsPublisher();

    bool publish(const proto::Setting& settings);
};

class SettingsSubscriber : private utils::Subscriber<proto::Setting> {
   public:
    SettingsSubscriber(const std::function<void(const proto::Setting&)>& callback);
};

}  // namespace rtt::net