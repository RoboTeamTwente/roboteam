// By: Floris Heinen
#pragma once

#include <google/protobuf/message.h>
#include <ssl_simulation_config.pb.h>
#include <ssl_simulation_control.pb.h>
#include <ssl_simulation_robot_control.pb.h>
#include <ssl_simulation_robot_feedback.pb.h>
#include <ssl_vision_geometry.pb.h>

#include <simulation/ConfigurationCommand.hpp>
#include <simulation/Feedback.hpp>
#include <simulation/RobotControlCommand.hpp>
#include <utilities.h>

#include <QtNetwork>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace rtt::robothub::simulation {
constexpr int DEFAULT_CONFIGURATION_PORT = 10300;
constexpr int DEFAULT_BLUE_CONTROL_PORT = 10301;
constexpr int DEFAULT_YELLOW_CONTROL_PORT = 10302;

typedef struct SimulatorNetworkConfiguration {
    QHostAddress simIpAddress = QHostAddress::LocalHost;

    // Ports the simulator listens to
    int blueControlPort = DEFAULT_BLUE_CONTROL_PORT;
    int yellowControlPort = DEFAULT_YELLOW_CONTROL_PORT;
    int configurationPort = DEFAULT_CONFIGURATION_PORT;

    // Ports we listen to for feedback (same as control according to ssl protocol)
    int blueFeedbackPort = DEFAULT_BLUE_CONTROL_PORT;
    int yellowFeedbackPort = DEFAULT_YELLOW_CONTROL_PORT;
    int configurationFeedbackPort = DEFAULT_CONFIGURATION_PORT;
} SimulatorNetworkConfiguration;

/*  This class can manage a connection with any simulator that follows the official SSL protocol.
    It can send robot control messages that control the robots, and it can send configuration
    messages that can configure the simulator, for example the size of the robots or physical
    properties of the field.
    To prevent waiting for a response, 3 listen threads are used to listen for feedback for
    the blue team, the yellow team and feedback for configuring the simulator. These threads will
    make a callback if it has been set. */
class SimulatorManager {
   public:
    // Can throw FailedToBindPortException
    SimulatorManager(SimulatorNetworkConfiguration configuration);
    ~SimulatorManager();

    // Both of the send functions return amount of bytes sent, return -1 if error occured
    int sendRobotControlCommand(RobotControlCommand& robotControlCommand, utils::TeamColor color);
    int sendConfigurationCommand(ConfigurationCommand& configurationCommand);

    // These will set the callback function when feedback has been received
    void setRobotControlFeedbackCallback(std::function<void(RobotControlFeedback&)> callback);
    void setConfigurationFeedbackCallback(std::function<void(ConfigurationFeedback&)> callback);

   private:
    SimulatorNetworkConfiguration networkConfiguration;

    QUdpSocket blueControlSocket;
    QUdpSocket yellowControlSocket;
    QUdpSocket configurationSocket;

    bool shouldStopListeningToFeedback;
    std::thread blueFeedbackListenThread;
    std::thread yellowFeedbackListenThread;
    std::thread configurationFeedbackListenThread;

    // Both blue and yellow feedback thread use same callback function, so mutex protects it
    std::mutex robotControlFeedbackMutex;
    std::function<void(RobotControlFeedback&)> robotControlFeedbackCallback;
    std::function<void(ConfigurationFeedback&)> configurationFeedbackCallback;

    // Returns the amount of bytes sent, returns -1 if error occured
    int sendPacket(google::protobuf::Message& packet, QUdpSocket& socket, int port);

    // These will call the callback functions, if set, whenever feedback is received
    void callRobotControlFeedbackCallback(RobotControlFeedback& feedback);
    void callConfigurationFeedbackCallback(ConfigurationFeedback& feedback);

    // These functions will keep listening for feedback until the Manager object gets destroyed
    void listenForRobotControlFeedback(utils::TeamColor color);
    void listenForConfigurationFeedback();

    // Will prevent threads from continuing and waits for them to finish
    void stopFeedbackListeningThreads();

    // These functions will convert a datagram of bytes into accessible data
    RobotControlFeedback getControlFeedbackFromDatagram(QNetworkDatagram& datagram, utils::TeamColor color);
    ConfigurationFeedback getConfigurationFeedbackFromDatagram(QNetworkDatagram& datagram);
};

class FailedToBindPortException : public std::exception {
   public:
    FailedToBindPortException(const std::string message);

    const char* what() const noexcept override;

   private:
    std::string message;
};
}  // namespace rtt::robothub::simulation