#pragma once

#include <google/protobuf/message.h>

#include <exception>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <utils/Channel.hpp>
#include <utils/Channels.hpp>
#include <zmqpp/reactor.hpp>
#include <zmqpp/zmqpp.hpp>

namespace rtt::net::utils {

    /* Defines a subscriber that subscribers to a TCP channel and forwards the message to a callback function/method
       There is no limit on how many subscribers can subscribe */
    class Subscriber {
    public:
        // Apply rule of 5: in all cases we cannot copy/move this object
        Subscriber(const Subscriber& copy) = delete;
        Subscriber& operator=(const Subscriber& other) = delete;
        Subscriber(const Subscriber&& copy) = delete;
        Subscriber& operator=(Subscriber&& data) = delete;

        /* Create a subscriber with a callback method that gets called when new data is available
         * @param channel: The channel to subscribe to
         * @param messageCallback: The function that gets the message forwarded to */
        explicit Subscriber(const ChannelType& channelType, const std::function<void(const std::string& message)> callback);
        ~Subscriber();

    private:
        Channel channel;
        zmqpp::context context;
        std::unique_ptr<zmqpp::socket> socket;
        std::unique_ptr<zmqpp::reactor> reactor;
        zmqpp::poller* poller;

        bool isPolling;
        std::thread pollingThread;

        const std::function<void(const std::string&)> messageCallback;

        // Gets called everytime a message is received so it can be forwared to the message callback
        void onMessageReceived();

        void poll();
    };

    class InvalidCallbackException : private std::exception {
    public:
        InvalidCallbackException(const std::string& message);

        const char* what() const noexcept override;

    private:
        const std::string message;
    };

}  // namespace rtt::net::utils