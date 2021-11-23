#pragma once
#include <google/protobuf/message.h>

#include <functional>
#include <mutex>
#include <string>
#include <zmqpp/reactor.hpp>
#include <zmqpp/zmqpp.hpp>

#include <utils/Channel.hpp>
#include <utils/Channels.hpp>

namespace rtt::net::utils {

/*
 * Defines a subscriber that subscribers to a TCP channel and forwards the message to a callback function/method
 * The type of the messages passed has to be specified beforehand in the template.
 *
 * This class is designed according to RAII: creating the object immediately subscribes to the channel,
 * and deleting the object immediately unsubscribes to the channel and cleans up memory.
 *
 * @author Lukas Bos
 * @date 26-10-2019
 */

template <class T_Response>
class Subscriber {
   private:
    Channel channel;
    zmqpp::context context;
    zmqpp::socket* socket;
    zmqpp::reactor* reactor;
    std::thread t1;
    bool running;

   public:
    // Apply rule of 5: in all cases we cannot copy/move this object
    Subscriber(const Subscriber& copy) = delete;
    Subscriber& operator=(const Subscriber& other) = delete;
    Subscriber(const Subscriber&& copy) = delete;
    Subscriber& operator=(Subscriber&& data) = delete;

   private:
    /*
     * Initialize the subscriber.
     * Bind to the socket on the given channel
     *
     * @param channel: the channel to subscribe to
     */
    void init(const ChannelType& channelType, std::string custom_ip = "") {
        this->channel = CHANNELS.at(channelType);
        this->reactor = new zmqpp::reactor();
        this->socket = new zmqpp::socket(this->context, zmqpp::socket_type::sub);
        this->socket->subscribe("");

        auto address = channel.getSubscribeAddress();
        if (!custom_ip.empty()) {
            std::cout << "Starting roboteam_proto subscriber with custom IP: " << custom_ip << std::endl;
            address = channel.getAddress(custom_ip, channel.port);
        }
        std::cout << "Starting roboteam_proto subscriber for channel " << channel.toInfoString(false) << std::endl;
        this->socket->connect(address);
        running = true;
    }

   public:
    /*
     * Create a subscriber with a callback method that gets called when new data is available
     * the new data will be available in the function.
     * this constructor can be used for free function calls
     *
     * @param channel: the channel to subscribe to
     * @param resp: A method pointer to a callback taking a reference to the specified response type
     * @param instance: the context of the method, i.e. a pointer to the class the method belongs to.
     */
    template <class T_Instance>
    Subscriber(const ChannelType& channelType, void (T_Instance::*subscriberCallbackMethod)(T_Response& resp), T_Instance* instance, std::string custom_ip = "") {
        init(channelType, custom_ip);

        zmqpp::poller* poller = &reactor->get_poller();
        auto callback = [=, this]() {
            if (poller->has_input(*socket)) {
                zmqpp::message response;
                T_Response output;

                socket->receive(response);
                if (output.ParseFromString(response.get(0))) {
                    (instance->*subscriberCallbackMethod)(output);  // call the subscriberCallback function
                } else {
                    std::cerr << "Received faulty protobuf packet in channel " << channel.toInfoString(false) << std::endl;
                }
            }
        };
        reactor->add(*socket, callback);
        t1 = std::thread(&Subscriber::poll, this);
    }

    /*
     * Create a subscriber with a callback function that gets called when new data is available
     * the new data will be available in the function.
     * this constructor can be used for free function calls
     *
     * @param channel: the channel to subscribe to
     * @param resp: A function pointer to a callback taking a reference to the specified response type
     */
    Subscriber(const ChannelType& channelType, void (*func)(T_Response& resp), std::string custom_ip = "") {
        init(channelType, custom_ip);

        zmqpp::poller* poller = &reactor->get_poller();
        auto callback = [=, this]() {
            if (poller->has_input(*socket)) {
                zmqpp::message response;
                T_Response output;

                socket->receive(response);
                if (output.ParseFromString(response.get(0))) {
                    func(output);  // call the subscriberCallback function
                } else {
                    std::cerr << "Received faulty protobuf packet in channel " << channel.toInfoString(false) << std::endl;
                }
            }
        };
        reactor->add(*socket, callback);
        t1 = std::thread(&Subscriber::poll, this);
    }

    /*
     * Deletes the subscriber.
     * First we make sure the polling thread stops and join the thread.
     * Then we safely close the socket and delete the pointers.
     */
    ~Subscriber() {
        std::cout << "Stopping roboteam_proto subscriber for channel " << channel.toInfoString(false) << std::endl;
        running = false;
        t1.join();
        reactor->remove(*socket);
        socket->close();
        delete socket;
        delete reactor;
    }

    /*
     * Poll for new messages with a timeout of 167 ms.
     * This practically means we evaluate the 'running' variable every 167ms
     * 167 ms is approximately equivalent to the time it takes to receive 10 packets on 60hz
     */
    void poll() {
        while (running) {
            reactor->poll(167);
        }
    }
};
}  // namespace rtt::net::utils