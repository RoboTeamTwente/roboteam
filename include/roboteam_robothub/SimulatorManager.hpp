// By: Floris Heinen
#pragma once

#include <google/protobuf/message.h>
#include <roboteam_proto/ssl_simulation_config.pb.h>
#include <roboteam_proto/ssl_simulation_control.pb.h>
#include <roboteam_proto/ssl_simulation_robot_control.pb.h>
#include <roboteam_proto/ssl_simulation_robot_feedback.pb.h>
#include <roboteam_proto/ssl_vision_geometry.pb.h>

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

/*  This class contains the information for moving robots.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Robots can be controlled by either wheel speeds, local velocities or global velocties.
    Local velocities are speeds like forward and sideway speeds, while global velocties are
    speeds relative to the field. */
class RobotControlCommand {
   public:
    void addRobotControlWithWheelSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float frontRightWheelVelocity, float backRightWheelVelocity,
                                        float backLeftWheelVelocity, float frontLeftWheelVelocity);
    void addRobotControlWithLocalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float forwardVelocity, float leftVelocity, float angularVelocity);
    void addRobotControlWithGlobalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float xVelocity, float yVelocity, float angularVelocity);
    proto::sim::RobotControl& getPacket();

   private:
    proto::sim::RobotControl controlCommand;
};

// Contains all the properties that can be changed in the simulator
typedef struct RobotProperties {
    // Units in meters, kilograms, degrees, m/s or m/s^2.
    float radius = 0.09f;
    float height = 0.15f;
    float mass = 2.0f;
    float maxKickSpeed = 6.5f;
    float maxChipSpeed = 6.5f;
    float centerToDribblerDistance = 0.09;
    // Robot limits
    float maxAcceleration = 3.0f;
    float maxAngularAcceleration = 3.0f;
    float maxDeceleration = 3.0f;
    float maxAngularDeceleration = 3.0f;
    float maxVelocity = 3.0f;
    float maxAngularVelocity = 3.0f;
    // Wheel angles. Counter-clockwise starting from dribbler
    float frontRightWheelAngle = 300.0f;
    float backRightWheelAngle = 210.0f;
    float backLeftWheelAngle = 150.0f;
    float frontLeftWheelAngle = 60.0f;
} RobotProperties;

/*  This class contains command information to configure and setup the simulator.
    It immediately stores the given information into a packet that can be sent to the simulator.
    Things to configure are robot positions, velocities, ball positions, physical properties of
    the robot, simulator speed, the vision port.
    TODO: Also add functionality to change the field geometry properties
    TODO: GrSim ignores robotId when setting robot properties, resulting in all robots from the
    same team having the same properties. This is grSim's fault, so go make a pull request to
    grSim that fixes this faulty behavior. */
class ConfigurationCommand {
   public:
    // VelocityInRolling transforms the velocities into spinning engergy.
    // ByForce will set velocities accordingly to make sure the ball ends
    // up at the specified coordinates.
    void setBallLocation(float x, float y, float z, float xVelocity, float yVelocity, float zVelocity, bool velocityInRolling, bool teleportSafely, bool byForce);
    // Orientation is a global rotation relative to the field.
    // shouldBePresentOnField will make a robot (dis)appear accordingly.
    void addRobotLocation(int id, bool isFromTeamYellow, float x, float y, float xVelocity, float yVelocity, float angularVelocity, float orientation, bool shouldBePresentOnField,
                          bool byForce);
    void setSimulationSpeed(float speed);
    void addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties);
    void setVisionPort(int port);

    proto::sim::SimulatorCommand& getPacket();

   private:
    proto::sim::SimulatorCommand configurationCommand;
};

typedef struct SimulationError {
    std::string code;
    std::string message;
} SimulationError;

typedef struct RobotControlFeedback {
    bool isTeamYellow;
    std::vector<int> robotsThatHaveTheBall;
    std::vector<SimulationError> simulationErrors;
} RobotControlFeedback;

typedef struct ConfigurationFeedback {
    std::vector<SimulationError> simulationErrors;
} ConfigurationFeedback;

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
    int sendRobotControlCommand(RobotControlCommand& robotControlCommand, bool isTeamYellow);
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
    void listenForRobotControlFeedback(bool listensForTeamYellow);
    void listenForConfigurationFeedback();

    // Will prevent threads from continuing and waits for them to finish
    void stopFeedbackListeningThreads();

    // These functions will convert a datagram of bytes into accessible data
    RobotControlFeedback getControlFeedbackFromDatagram(QNetworkDatagram& datagram, bool isTeamYellow);
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