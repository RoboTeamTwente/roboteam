#include "Handler.h"

#include <roboteam_utils/Timer.h>
#include <sstream>
#include <roboteam_proto/messages_robocup_ssl_wrapper.pb.h>

void Handler::start() {
    init();
    setupSSLClients();

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
            observer.process(vision_packets,referee_packets,robothub_info);
            auto state = observer.getState(Time::now());//TODO: fix time extrapolation
            while(!pub_state->send(state)); //TODO: try sending for a max limit amount of times
        },
        100);
}

void Handler::init() {
    pub_state = std::make_unique<proto::Publisher<proto::State>>(proto::WORLD_CHANNEL);
    //TODO: update channel type
    sub_feedback = std::make_unique<proto::Subscriber<proto::RobotData>>(proto::FEEDBACK_PRIMARY_CHANNEL,&Handler::robotDataCallBack,this);
}

void Handler::setupSSLClients() {
    constexpr int DEFAULT_VISION_PORT = 10006;
    constexpr int DEFAULT_REFEREE_PORT = 10003;

    const string SSL_VISION_SOURCE_IP = "224.5.23.2";
    const string SSL_REFEREE_SOURCE_IP = "224.5.23.1";
    
    vision_client = std::make_unique<RoboCupSSLClient>(DEFAULT_VISION_PORT, SSL_VISION_SOURCE_IP);
    referee_client = std::make_unique<RoboCupSSLClient>(DEFAULT_REFEREE_PORT, SSL_REFEREE_SOURCE_IP);

    cout << "Vision  : " << SSL_VISION_SOURCE_IP << ":" << DEFAULT_VISION_PORT << endl;
    cout << "Referee  : " << SSL_REFEREE_SOURCE_IP << ":" << DEFAULT_REFEREE_PORT << endl;

    vision_client->open(false);  // boolean blocking
    referee_client->open(false);
    this_thread::sleep_for(chrono::microseconds(10000));
}

std::vector<proto::SSL_WrapperPacket> Handler::receiveVisionPackets() {
    std::vector<proto::SSL_WrapperPacket> receivedPackets;
    proto::SSL_WrapperPacket packet;
    while(vision_client && vision_client->receive(packet)){
        receivedPackets.push_back(packet);
    }
    return receivedPackets;
}
std::vector<proto::SSL_Referee> Handler::receiveRefereePackets()  {
    std::vector<proto::SSL_Referee> receivedPackets;
    proto::SSL_Referee packet;
    while(referee_client && referee_client->receive(packet)){
        receivedPackets.push_back(packet);
    }
    return receivedPackets;
}

void Handler::robotDataCallBack(proto::RobotData& data) {
    std::lock_guard guard(sub_mutex);
    receivedRobotData.push_back(data);
}

