#pragma once

#include <utils/Channel.hpp>
#include <utils/Channels.hpp>

#include <google/protobuf/message.h>
#include <zmqpp/reactor.hpp>
#include <zmqpp/zmqpp.hpp>
#include <functional>
#include <string>
#include <thread>
#include <memory>

namespace rtt::net::utils {

/* Defines a subscriber that subscribers to a TCP channel and forwards the message to a callback function/method */
template<class Message>
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
    explicit Subscriber(const ChannelType& channelType, const std::function<void(const Message& message)> callback)
        : messageCallback(callback) {
        // Initialize objects
        this->channel = CHANNELS.at(channelType);
        this->reactor = std::make_unique<zmqpp::reactor>();
        this->socket = std::make_unique<zmqpp::socket>(this->context, zmqpp::socket_type::sub);
        this->socket->subscribe(""); // TODO: What does this do?

        auto address = channel.getSubscribeAddress();
        this->socket->connect(address);

        this->poller = &reactor->get_poller();
        auto callback2 = std::bind(&Subscriber::onMessageReceived, this);
        this->reactor->add(*socket.get(), callback2);

        this->isPolling = true;
        this->pollingThread = std::thread(&Subscriber::poll, this);
    }
    ~Subscriber() {
        this->isPolling = false;
        if (this->pollingThread.joinable()) { this->pollingThread.join(); }
    }

private:
    Channel channel;
    zmqpp::context context;
    std::unique_ptr<zmqpp::socket> socket;
    std::unique_ptr<zmqpp::reactor> reactor;
    zmqpp::poller* poller;

    bool isPolling;
    std::thread pollingThread;

    const std::function<void(const Message&)> messageCallback;

    // Gets called everytime a message is received so it can be forwared to the message callback
    void onMessageReceived() {
        if (this->poller->has_input(*this->socket)) {
            zmqpp::message response;
            socket->receive(response);
            
            Message message;
            message.ParseFromString(response.get(0));

            this->messageCallback(message);
        }
    }

    void poll() {
        while (this->isPolling) {
            this->reactor->poll(167);
        }
    }
};
}  // namespace rtt::net::utils