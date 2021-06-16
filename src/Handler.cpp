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

            auto state = observer.process(Time::now(),vision_packets,referee_packets,robothub_info); //TODO: fix time extrapolation
            while(!pub_state->send(state)); //TODO: try sending for a max limit amount of times
        },
        100);
}

void Handler::init() {
    pub_state = std::make_unique<proto::Publisher<proto::State>>(proto::WORLD_CHANNEL);
    //TODO: update channel type
    sub_feedback = std::make_unique<proto::Subscriber<proto::RobotData>>(proto::FEEDBACK_PRIMARY_CHANNEL,&Handler::robotDataCallBack,this);
    sub_feedback_2 = std::make_unique<proto::Subscriber<proto::RobotData>>(proto::FEEDBACK_SECONDARY_CHANNEL,&Handler::robotDataCallBack,this);

}

void Handler::setupSSLClients() {
    constexpr quint16 DEFAULT_VISION_PORT = 10006;
    constexpr quint16 DEFAULT_REFEREE_PORT = 10003;

    const QString SSL_VISION_SOURCE_IP = "224.5.23.2";
    const QString SSL_REFEREE_SOURCE_IP = "224.5.23.1";
    
    vision_client = new RobocupReceiver<proto::SSL_WrapperPacket>(QHostAddress(SSL_VISION_SOURCE_IP),DEFAULT_VISION_PORT);
    referee_client = new RobocupReceiver<proto::SSL_Referee>(QHostAddress(SSL_REFEREE_SOURCE_IP),DEFAULT_REFEREE_PORT);

    cout << "Vision  : " << SSL_VISION_SOURCE_IP.toStdString() << ":" << DEFAULT_VISION_PORT << endl;
    cout << "Referee  : " << SSL_REFEREE_SOURCE_IP.toStdString() << ":" << DEFAULT_REFEREE_PORT << endl;

    vision_client->connect();
    referee_client->connect();
    this_thread::sleep_for(chrono::microseconds(10000));
}

std::vector<proto::SSL_WrapperPacket> Handler::receiveVisionPackets() {
    std::vector<proto::SSL_WrapperPacket> receivedPackets;
    bool ok = vision_client->receive(receivedPackets);
    if(!ok){
      std::cout<<"error receiving vision messages"<<std::endl;
    }
    return receivedPackets;
}
std::vector<proto::SSL_Referee> Handler::receiveRefereePackets()  {
  std::vector<proto::SSL_Referee> receivedPackets;
  bool ok = referee_client->receive(receivedPackets);
  if(!ok){
    std::cout<<"error receiving referee messages"<<std::endl;
  }
  return receivedPackets;
}

void Handler::robotDataCallBack(proto::RobotData& data) {
    std::lock_guard guard(sub_mutex);
    receivedRobotData.push_back(data);
}
Handler::~Handler() {
  delete referee_client;
  delete vision_client;
}

