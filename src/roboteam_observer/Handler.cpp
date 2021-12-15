#include "Handler.h"

#include <proto/messages_robocup_ssl_wrapper.pb.h>
#include <roboteam_utils/Timer.h>

#include <sstream>

void Handler::start() {
    if (!initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }
    if (!this->setupSSLClients()) {
        throw FailedToSetupSSLClients();
    }

    roboteam_utils::Timer t;

    t.loop(
        [&]() {
            auto vision_packets = receiveVisionPackets();
            auto referee_packets = receiveRefereePackets();
            std::vector<proto::RobotData> robothub_info;
            {
                std::lock_guard guard(sub_mutex);
                robothub_info = receivedRobotData;
                receivedRobotData.clear();
            }

            auto state = observer.process(Time::now(), vision_packets, referee_packets, robothub_info);  // TODO: fix time extrapolation
            
            this->worldPublisher->publish(state);
            // TODO: Try resending if failed to send message
        },
        100);
}

bool Handler::initializeNetworkers() {
    this->worldPublisher = std::make_unique<rtt::net::WorldPublisher>();

    auto feedbackCallback = std::bind(&Handler::robotDataCallBack, this, std::placeholders::_1);
    this->feedbackSubscriber = std::make_unique<rtt::net::RobotFeedbackSubscriber>(feedbackCallback);

    return this->worldPublisher != nullptr
        && this->feedbackSubscriber != nullptr;
}

bool Handler::setupSSLClients() {
    bool success = true;
    constexpr quint16 DEFAULT_VISION_PORT = 10006;
    constexpr quint16 DEFAULT_REFEREE_PORT = 10003;

    const QString SSL_VISION_SOURCE_IP = "224.5.23.2";
    const QString SSL_REFEREE_SOURCE_IP = "224.5.23.1";

    this->vision_client = std::make_unique<RobocupReceiver<proto::SSL_WrapperPacket>>(QHostAddress(SSL_VISION_SOURCE_IP), DEFAULT_VISION_PORT);
    this->referee_client = std::make_unique<RobocupReceiver<proto::SSL_Referee>>(QHostAddress(SSL_REFEREE_SOURCE_IP), DEFAULT_REFEREE_PORT);
    success = vision_client != nullptr && referee_client != nullptr;

    cout << "Vision  : " << SSL_VISION_SOURCE_IP.toStdString() << ":" << DEFAULT_VISION_PORT << endl;
    cout << "Referee  : " << SSL_REFEREE_SOURCE_IP.toStdString() << ":" << DEFAULT_REFEREE_PORT << endl;

    success = success && vision_client->connect();
    success = success && referee_client->connect();

    this_thread::sleep_for(chrono::microseconds(10000));

    return success;
}

std::vector<proto::SSL_WrapperPacket> Handler::receiveVisionPackets() {
    std::vector<proto::SSL_WrapperPacket> receivedPackets;
    bool ok = vision_client->receive(receivedPackets);
    if (!ok) {
        std::cout << "error receiving vision messages" << std::endl;
    }
    return receivedPackets;
}
std::vector<proto::SSL_Referee> Handler::receiveRefereePackets() {
    std::vector<proto::SSL_Referee> receivedPackets;
    bool ok = referee_client->receive(receivedPackets);
    if (!ok) {
        std::cout << "error receiving referee messages" << std::endl;
    }
    return receivedPackets;
}

void Handler::robotDataCallBack(const proto::RobotData& data) {
    std::lock_guard guard(sub_mutex);
    receivedRobotData.push_back(data);
}

const char* FailedToInitializeNetworkersException::what() const throw() {
    return "Failed to initialize networker(s). Is another observer running?";
}
const char* FailedToSetupSSLClients::what() const throw() {
    return "Failed to setup SSL client(s). Is another observer running?";
}