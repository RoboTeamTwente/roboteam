#ifndef WORLDHANDLER_H
#define WORLDHANDLER_H

#include <net/robocup_ssl_client.h>
#include <observer/Observer.h>

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
    std::unique_ptr<RobocupReceiver<proto::SSL_Referee>> referee_client;

    Observer observer;
    std::vector<proto::RobotData> receivedRobotData;
    std::mutex sub_mutex;

   public:
    Handler() = default;

    /*
     * Setup a world with a kalmanfilter, and initialize the publishers for publishing data.
     */
    bool initializeNetworkers();
    bool setupSSLClients();

    void start();
    std::vector<proto::SSL_WrapperPacket> receiveVisionPackets();
    std::vector<proto::SSL_Referee> receiveRefereePackets();
    void robotDataCallBack(const proto::RobotData& data);
};

class FailedToInitializeNetworkersException : public std::exception {
    const char* what() const throw();
};
class FailedToSetupSSLClients : public std::exception {
    const char* what() const throw();
};

#endif