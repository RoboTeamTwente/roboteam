#include <utils/Subscriber.hpp>

namespace rtt::net::utils {

void Subscriber::init(const ChannelType& channelType, std::string custom_ip = "") {
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

Subscriber::Subscriber(const ChannelType& channelType, void (T_Instance::*subscriberCallbackMethod)(T_Response& resp), T_Instance* instance, std::string custom_ip = "") {
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

Subscriber::~Subscriber() {
    std::cout << "Stopping roboteam_proto subscriber for channel " << channel.toInfoString(false) << std::endl;
    running = false;
    t1.join();
    reactor->remove(*socket);
    socket->close();
    delete socket;
    delete reactor;
}

Subscriber::poll() {
    while (running) {
        reactor->poll(167);
    }
}

} // namespace rtt::net::utils