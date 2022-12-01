#include "Handler.h"

#include <roboteam_utils/Timer.h>
#include <sstream>
#include <algorithm>
#include <proto/messages_robocup_ssl_wrapper.pb.h>

#include <roboteam_logging/LogFileWriter.h>

void Handler::start(bool shouldLog) {
    if (!initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }
    if (!this->setupSSLClients()) {
        throw FailedToSetupSSLClients();
    }

    roboteam_utils::Timer t;

    std::optional<rtt::LogFileWriter> fileWriter = std::nullopt;

    if (shouldLog) {
        auto now = Time::now();
        long total_seconds = now.asIntegerSeconds();
        long days = total_seconds/(24*3600);
        long day_seconds = total_seconds % (24*3600);
        long hours = day_seconds/3600;
        long minutes = (day_seconds -hours*3600)/60;
        long seconds = day_seconds % 60;
        long mili = now.asIntegerMilliSeconds() % 1000;
        std::string file_name = "world_log_" + std::to_string(days) + "_" +std::to_string(hours) + "_" + std::to_string(minutes) +"_" + std::to_string(seconds) + "_" + std::to_string(mili)+".log";
        fileWriter = rtt::LogFileWriter();
        fileWriter.value().open(file_name);
    }

    t.loop(
        [&]() {
            auto vision_packets = receiveVisionPackets();
            auto referee_packets = receiveRefereePackets();
            std::vector<rtt::RobotsFeedback> robothub_info;
            {
                std::lock_guard guard(sub_mutex);
                std::swap(robothub_info, this->receivedRobotData);
            }

            auto state = observer.process(vision_packets,referee_packets,robothub_info); //TODO: fix time extrapolation
            std::size_t iterations = 0;
            bool sent = false;
            while(iterations < 10){
              auto bytesSent = worldPublisher->publish(state);
              if(bytesSent > 0){
                sent = true;
                break;
              }
              iterations++;
            }
            if(!sent){
              std::cout<<"could not send data on publisher!"<<std::endl;
            }
            
            if (fileWriter.has_value()) {
                bool good = fileWriter.value().addMessage(state,Time::now().asNanoSeconds());
                if(!good){
                    std::cout<<"could not log message to file!\n";
                }
            }
        },
        100);
}
bool Handler::initializeNetworkers() {
    this->worldPublisher = std::make_unique<rtt::net::WorldPublisher>();

    this->feedbackSubscriber = std::make_unique<rtt::net::RobotFeedbackSubscriber>([&](const rtt::RobotsFeedback& feedback) {
        onRobotFeedback(feedback);
    });

    return this->worldPublisher != nullptr && this->feedbackSubscriber != nullptr;
}

bool Handler::setupSSLClients() {
    bool success = true;
    constexpr quint16 DEFAULT_VISION_PORT = 10006;
    constexpr quint16 DEFAULT_REFEREE_PORT = 10003;

    const QString SSL_VISION_SOURCE_IP = "224.5.23.2";
    const QString SSL_REFEREE_SOURCE_IP = "224.5.23.1";
    
    this->vision_client = std::make_unique<RobocupReceiver<proto::SSL_WrapperPacket>>(QHostAddress(SSL_VISION_SOURCE_IP),DEFAULT_VISION_PORT);
    this->referee_client = std::make_unique<RobocupReceiver<proto::SSL_Referee>>(QHostAddress(SSL_REFEREE_SOURCE_IP),DEFAULT_REFEREE_PORT);

    success = vision_client != nullptr && referee_client != nullptr;
    std::cout << "Vision  : " << SSL_VISION_SOURCE_IP.toStdString() << ":" << DEFAULT_VISION_PORT << std::endl;
    std::cout << "Referee  : " << SSL_REFEREE_SOURCE_IP.toStdString() << ":" << DEFAULT_REFEREE_PORT << std::endl;

    success &= vision_client->connect();
    success &= referee_client->connect();
    std::this_thread::sleep_for(std::chrono::microseconds(10000));

    return success;
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

void Handler::onRobotFeedback(const rtt::RobotsFeedback& feedback) {
    std::lock_guard guard(sub_mutex);
    receivedRobotData.push_back(feedback);
}

const char* FailedToInitializeNetworkersException::what() const noexcept(true) { return "Failed to initialize networker(s). Is another observer running?"; }
const char* FailedToSetupSSLClients::what() const noexcept(true){ return "Failed to setup SSL client(s). Is another observer running?"; }