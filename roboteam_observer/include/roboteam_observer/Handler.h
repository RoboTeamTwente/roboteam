#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H

#include <observer/Observer.h>
#include <proto/State.pb.h>
#include <roboteam_logging/LogFileReader.h>
#include <roboteam_logging/LogFileWriter.h>

#include <RobotFeedbackNetworker.hpp>
#include <WorldNetworker.hpp>
#include <exception>
#include <memory>
#include <utility>
#include <vector>

#include "RobocupReceiver.h"

class Handler {
   private:
    std::unique_ptr<rtt::net::RobotFeedbackSubscriber> feedbackSubscriber;
    std::unique_ptr<rtt::net::WorldPublisher> worldPublisher;

    std::unique_ptr<RobocupReceiver<proto::SSL_WrapperPacket>> vision_client;
    std::unique_ptr<RobocupReceiver<proto::Referee>> referee_client;

    Observer observer;
    std::vector<rtt::RobotsFeedback> receivedRobotData;
    std::mutex sub_mutex;

    static std::optional<rtt::LogFileWriter> fileWriter;

   public:
    Handler() = default;

    /*
     * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
     */
    bool initializeNetworkers();
    bool setupSSLClients(std::string visionip, std::string refereeip, int visionport, int refereeport);

    void startReplay(rtt::LogFileReader& reader);
    void start(std::string visionip, std::string refereeip, int visionport, int refereeport, bool shouldLog = false, const std::vector<int>& camera_ids = {});
    std::vector<proto::SSL_WrapperPacket> receiveVisionPackets();
    std::vector<proto::Referee> receiveRefereePackets();
    void onRobotFeedback(const rtt::RobotsFeedback& feedback);
};

class FailedToInitializeNetworkersException : public std::exception {
    const char* what() const throw();
};
class FailedToSetupSSLClients : public std::exception {
    const char* what() const throw();
};

#endif