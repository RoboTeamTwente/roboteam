#pragma once
#include <roboteam_utils/AIData.hpp>
#include <functional>
#include <utils/Publisher.hpp>
#include <utils/Subscriber.hpp>

namespace rtt::net {

class AIYellowDataPublisher : private utils::Publisher {
   public:
    AIYellowDataPublisher();

    // Publishes the given AI data. Returns success
    bool publish(const AIData& data);
};

class AIYellowDataSubscriber : private utils::Subscriber {
   public:
    AIYellowDataSubscriber(const std::function<void(const AIData&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const AIData&)> callback;
};

class AIBlueDataPublisher : private utils::Publisher {
   public:
    AIBlueDataPublisher();

    // Publishes the given AI data. Returns success
    bool publish(const AIData& data);
};

class AIBlueDataSubscriber : private utils::Subscriber {
   public:
    AIBlueDataSubscriber(const std::function<void(const AIData&)>& callback);

   private:
    void onPublishedMessage(const std::string& message);
    const std::function<void(const AIData&)> callback;
};

}  // namespace rtt::net