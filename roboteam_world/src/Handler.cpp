#include "Handler.h"

#include <roboteam_utils/Timer.h>
#include <sstream>
#include <algorithm>
#include <proto/messages_robocup_ssl_wrapper.pb.h>

#include <roboteam_logging/LogFileWriter.h>

void Handler::start(std::string visionip, std::string refereeip, int visionport, int refereeport, bool shouldLog) {
    if (!initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }
    if (!this->setupSSLClients(visionip, refereeip, visionport, refereeport)) {
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

    // Feedback for user
    int no_vision_received = 0;
    bool receiving_vision = true;
    std::cout << std::endl << "\rReceiving vision packets" << std::flush;

    t.loop(
        [&]() {
            auto vision_packets = receiveVisionPackets();
            auto referee_packets = receiveRefereePackets();

            // Feedback to user
            if (vision_packets.empty()) {
                no_vision_received++;
                receiving_vision = no_vision_received < 100;
                if (0 < no_vision_received && no_vision_received % 50 == 0) {
                    std::cout << "\rNot receiving vision packets ";
                    // Print spinner to show that program is still running
                         if(no_vision_received % 200 <  50) std::cout << "|";
                    else if(no_vision_received % 200 < 100) std::cout << "/";
                    else if(no_vision_received % 200 < 150) std::cout << "-";
                    else if(no_vision_received % 200 < 200) std::cout << "\\";
                    std::cout << std::flush;
                }
            } else {
                // Only print this once. If we're receiving packets, we don't want to influence processing time with slow std::cout calls
                if(!receiving_vision){
                    std::cout << "\rReceiving vision packets        \r" << std::flush;
                    receiving_vision = true;
                }
                no_vision_received = 0;
            }
            
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

bool Handler::setupSSLClients(std::string visionip, std::string refereeip, int visionport, int refereeport) {
    std::cout << "Vision  : " << visionip << ":" << visionport << std::endl;
    std::cout << "Referee : " << refereeip << ":" << refereeport << std::endl;
    
    bool success = true;

    QHostAddress visionAddress(QString::fromStdString(visionip));
    QHostAddress refereeAddress(QString::fromStdString(refereeip));

    success &= !(visionAddress.isNull());
    success &= !(refereeAddress.isNull());

    if(visionAddress.isNull()) std::cout << "Error! Invalid vision ip-address: " << visionip << std::endl;
    if(refereeAddress.isNull()) std::cout << "Error! Invalid referee ip-address: " << refereeip << std::endl;

    this->vision_client = std::make_unique<RobocupReceiver<proto::SSL_WrapperPacket>>(visionAddress, visionport);
    this->referee_client = std::make_unique<RobocupReceiver<proto::SSL_Referee>>(refereeAddress, refereeport);

    success &= vision_client != nullptr && referee_client != nullptr;
    
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