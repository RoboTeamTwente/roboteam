#include <utils/Subscriber.hpp>

namespace rtt::net::utils {

Subscriber::Subscriber(const ChannelType& channelType, std::function<void(const std::string&)> messageCallback) {
    // Initialize objects
    this->channel = CHANNELS.at(channelType);
    this->reactor = std::make_unique<zmqpp::reactor>();
    this->socket = std::make_unique<zmqpp::socket>(this->context, zmqpp::socket_type::sub);
    this->socket->subscribe(""); // TODO: What does this do?

    auto address = channel.getSubscribeAddress();
    this->socket->connect(address);

    this->poller = &reactor->get_poller();
    auto callback = std::bind(&Subscriber::onMessageReceived, this);
    this->reactor->add(*socket.get(), callback);

    this->isPolling = true;
    this->pollingThread = std::thread(&Subscriber::poll, this);
}

Subscriber::~Subscriber() {
    this->isPolling = false;
    if (this->pollingThread.joinable()) { this->pollingThread.join(); }
}

void Subscriber::onMessageReceived() {
    if (this->poller->has_input(*this->socket)) {
        zmqpp::message response;
        socket->receive(response);

        this->messageCallback(response.get(0));
    }
}

void Subscriber::poll() {
    while (this->isPolling) {
        this->reactor->poll(167);
    }
}

} // namespace rtt::net::utils