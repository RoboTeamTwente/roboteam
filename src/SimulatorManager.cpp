// By: Floris Heinen
#include "SimulatorManager.hpp"

namespace rtt::robothub::simulation {

SimulatorManager::SimulatorManager(SimulatorNetworkConfiguration config) {
    this->networkConfiguration = config;

    // Bind sockets so we receive feedback
    if (!this->blueControlSocket.bind(QHostAddress::AnyIPv4, config.blueFeedbackPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
        throw FailedToBindPortException("Failed to bind to blue feedback port(" + std::to_string(config.blueFeedbackPort) + "). Is it bound by another program?");
    if (!this->yellowControlSocket.bind(QHostAddress::AnyIPv4, config.yellowFeedbackPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
        throw FailedToBindPortException("Failed to bind to yellow feedback port(" + std::to_string(config.yellowFeedbackPort) + "). Is it bound by another program?");
    if (!this->configurationSocket.bind(QHostAddress::AnyIPv4, config.configurationFeedbackPort, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint))
        throw FailedToBindPortException("Failed to bind to control feedback port(" + std::to_string(config.configurationFeedbackPort) + "). Is it bound by another program?");

    this->shouldStopListeningToFeedback = false;

    // Spawn listening threads that handle incoming feedback
    this->blueFeedbackListenThread = std::thread([this] { listenForRobotControlFeedback(false); });
    this->yellowFeedbackListenThread = std::thread([this] { listenForRobotControlFeedback(true); });
    this->configurationFeedbackListenThread = std::thread([this] { listenForConfigurationFeedback(); });
}

SimulatorManager::~SimulatorManager() { this->stopFeedbackListeningThreads(); }

int SimulatorManager::sendRobotControlCommand(RobotControlCommand& robotControlCommand, bool forTeamYellow) {
    int bytesSent;

    if (forTeamYellow) {
        bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->yellowControlSocket, this->networkConfiguration.yellowControlPort);
    } else {
        bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->blueControlSocket, this->networkConfiguration.blueControlPort);
    }

    return bytesSent;
}

int SimulatorManager::sendConfigurationCommand(ConfigurationCommand& configurationCommand) {
    int bytesSent = this->sendPacket(configurationCommand.getPacket(), this->configurationSocket, this->networkConfiguration.configurationPort);
    return bytesSent;
}

void SimulatorManager::setRobotControlFeedbackCallback(std::function<void(RobotControlFeedback&)> callback) { this->robotControlFeedbackCallback = callback; }
void SimulatorManager::setConfigurationFeedbackCallback(std::function<void(ConfigurationFeedback&)> callback) { this->configurationFeedbackCallback = callback; }

int SimulatorManager::sendPacket(google::protobuf::Message& packet, QUdpSocket& socket, int port) {
    int bytesSent;

    // Create a byteArray where the packet can be serialized into
    QByteArray datagram;
    datagram.resize(packet.ByteSizeLong());
    packet.SerializeToArray(datagram.data(), datagram.size());
    // Send contents of byteArray to simulator
    bytesSent = socket.writeDatagram(datagram, this->networkConfiguration.simIpAddress, port);

    return bytesSent;
}

void SimulatorManager::callRobotControlFeedbackCallback(RobotControlFeedback& feedback) {
    // Feedback for blue and yellow use the same callback, so use mutex to prevent data race
    std::scoped_lock functionLock(this->robotControlFeedbackMutex);
    // Only call if the callback function has been set
    if (this->robotControlFeedbackCallback != nullptr) this->robotControlFeedbackCallback(feedback);
}

void SimulatorManager::callConfigurationFeedbackCallback(ConfigurationFeedback& feedback) {
    // Only call if the callback function has been set
    if (this->configurationFeedbackCallback != nullptr) this->configurationFeedbackCallback(feedback);
}

void SimulatorManager::listenForRobotControlFeedback(bool listenForTeamYellow) {
    // Select the socket that is used by the team
    QUdpSocket& teamSocket = listenForTeamYellow ? this->yellowControlSocket : this->blueControlSocket;

    while (!this->shouldStopListeningToFeedback) {
        while (teamSocket.hasPendingDatagrams()) {
            QNetworkDatagram datagram = teamSocket.receiveDatagram();
            if (datagram.isValid()) {
                RobotControlFeedback f = this->getControlFeedbackFromDatagram(datagram, listenForTeamYellow);
                this->callRobotControlFeedbackCallback(f);
            }
        }
    }
}

void SimulatorManager::listenForConfigurationFeedback() {
    while (!this->shouldStopListeningToFeedback) {
        while (this->configurationSocket.hasPendingDatagrams()) {
            QNetworkDatagram datagram = this->configurationSocket.receiveDatagram();
            if (datagram.isValid()) {
                ConfigurationFeedback f = this->getConfigurationFeedbackFromDatagram(datagram);
                this->configurationFeedbackCallback(f);
            }
        }
    }
}

RobotControlFeedback SimulatorManager::getControlFeedbackFromDatagram(QNetworkDatagram& datagram, bool isTeamYellow) {
    // First, parse datagram into the proto packet that it resembles
    proto::sim::RobotControlResponse response;
    response.ParseFromArray(datagram.data().data(), datagram.data().size());

    // Then, put the information in an easy accessible struct
    RobotControlFeedback feedback;
    feedback.isTeamYellow = isTeamYellow;

    for (const proto::sim::RobotFeedback& robotFeedback : response.feedback()) {
        // This boolean is optional, so first check if it has been set before checking if it is set to true
        if (robotFeedback.has_dribbler_ball_contact() && robotFeedback.dribbler_ball_contact()) feedback.robotsThatHaveTheBall.push_back(robotFeedback.id());
    }

    for (const proto::sim::SimulatorError& simulatorError : response.errors()) {
        SimulationError error;
        if (simulatorError.has_code()) error.code = simulatorError.code();
        if (simulatorError.has_message()) error.message = simulatorError.message();
        feedback.simulationErrors.push_back(error);
    }

    return feedback;
}

ConfigurationFeedback SimulatorManager::getConfigurationFeedbackFromDatagram(QNetworkDatagram& datagram) {
    // First, parse datagram into the proto packet that it resembles
    proto::sim::SimulatorResponse response;
    response.ParseFromArray(datagram.data().data(), datagram.data().size());

    // Then, put the information in an easy accesible struct
    ConfigurationFeedback feedback;
    for (const proto::sim::SimulatorError& simulationError : response.errors()) {
        SimulationError error;
        if (simulationError.has_code()) error.code = simulationError.code();
        if (simulationError.has_message()) error.message = simulationError.message();
        feedback.simulationErrors.push_back(error);
    }
    return feedback;
}

void SimulatorManager::stopFeedbackListeningThreads() {
    this->shouldStopListeningToFeedback = true;

    if (this->blueFeedbackListenThread.joinable()) this->blueFeedbackListenThread.join();

    if (this->yellowFeedbackListenThread.joinable()) this->yellowFeedbackListenThread.join();

    if (this->configurationFeedbackListenThread.joinable()) this->configurationFeedbackListenThread.join();

    this->shouldStopListeningToFeedback = false;
}

void ConfigurationCommand::setBallLocation(float x, float y, float z, float xVelocity, float yVelocity, float zVelocity, bool velocityInRolling, bool teleportSafely,
                                           bool byForce) {
    proto::sim::TeleportBall* tpBallCommand = this->configurationCommand.mutable_control()->mutable_teleport_ball();
    tpBallCommand->set_x(x);
    tpBallCommand->set_y(y);
    tpBallCommand->set_z(z);
    tpBallCommand->set_vx(xVelocity);
    tpBallCommand->set_vy(yVelocity);
    tpBallCommand->set_vz(zVelocity);
    tpBallCommand->set_roll(velocityInRolling);
    tpBallCommand->set_teleport_safely(teleportSafely);
    tpBallCommand->set_by_force(byForce);
}
void ConfigurationCommand::addRobotLocation(int id, bool isFromTeamYellow, float x, float y, float xVelocity, float yVelocity, float angularVelocity, float orientation,
                                            bool shouldBePresentOnField, bool byForce) {
    proto::sim::TeleportRobot* tpRobotCommand = this->configurationCommand.mutable_control()->add_teleport_robot();
    tpRobotCommand->mutable_id()->set_id(id);
    tpRobotCommand->mutable_id()->set_team(isFromTeamYellow ? proto::sim::Team::YELLOW : proto::sim::Team::BLUE);
    tpRobotCommand->set_x(x);
    tpRobotCommand->set_y(y);
    tpRobotCommand->set_v_x(xVelocity);
    tpRobotCommand->set_v_y(yVelocity);
    tpRobotCommand->set_v_angular(angularVelocity);
    tpRobotCommand->set_orientation(orientation);
    tpRobotCommand->set_present(shouldBePresentOnField);
    tpRobotCommand->set_by_force(byForce);
}
void ConfigurationCommand::setSimulationSpeed(float speed) { this->configurationCommand.mutable_control()->set_simulation_speed(speed); }
void ConfigurationCommand::addRobotSpecs(int id, bool isFromTeamYellow, RobotProperties& robotProperties) {
    proto::sim::RobotSpecs* specs = this->configurationCommand.mutable_config()->add_robot_specs();

    specs->mutable_id()->set_id(id);
    specs->mutable_id()->set_team(isFromTeamYellow ? proto::sim::Team::YELLOW : proto::sim::Team::BLUE);
    specs->set_radius(robotProperties.radius);
    specs->set_height(robotProperties.height);
    specs->set_mass(robotProperties.mass);
    specs->set_max_linear_kick_speed(robotProperties.maxKickSpeed);
    specs->set_max_chip_kick_speed(robotProperties.maxChipSpeed);
    specs->set_center_to_dribbler(robotProperties.centerToDribblerDistance);
    // Set limits
    specs->mutable_limits()->set_acc_speedup_absolute_max(robotProperties.maxAcceleration);
    specs->mutable_limits()->set_acc_speedup_angular_max(robotProperties.maxAngularAcceleration);
    specs->mutable_limits()->set_acc_brake_absolute_max(robotProperties.maxDeceleration);
    specs->mutable_limits()->set_acc_brake_angular_max(robotProperties.maxAngularDeceleration);
    specs->mutable_limits()->set_vel_absolute_max(robotProperties.maxVelocity);
    specs->mutable_limits()->set_vel_angular_max(robotProperties.maxAngularVelocity);
    // Set wheel angles
    specs->mutable_wheel_angles()->set_front_right(robotProperties.frontRightWheelAngle);
    specs->mutable_wheel_angles()->set_back_right(robotProperties.backRightWheelAngle);
    specs->mutable_wheel_angles()->set_back_left(robotProperties.backLeftWheelAngle);
    specs->mutable_wheel_angles()->set_front_left(robotProperties.frontLeftWheelAngle);
}
void ConfigurationCommand::setVisionPort(int port) { this->configurationCommand.mutable_config()->set_vision_port(port); }
proto::sim::SimulatorCommand& ConfigurationCommand::getPacket() { return this->configurationCommand; }

void RobotControlCommand::addRobotControlWithWheelSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float frontRightWheelVelocity,
                                                         float backRightWheelVelocity, float backLeftWheelVelocity, float frontLeftWheelVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveWheelVelocity* velocity = command->mutable_move_command()->mutable_wheel_velocity();
    velocity->set_front_right(frontRightWheelVelocity);
    velocity->set_back_right(backRightWheelVelocity);
    velocity->set_back_left(backLeftWheelVelocity);
    velocity->set_front_left(frontLeftWheelVelocity);
}
void RobotControlCommand::addRobotControlWithLocalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float forwardVelocity, float leftVelocity,
                                                         float angularVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveLocalVelocity* velocity = command->mutable_move_command()->mutable_local_velocity();
    velocity->set_forward(forwardVelocity);
    velocity->set_left(leftVelocity);
    velocity->set_angular(angularVelocity);
}
void RobotControlCommand::addRobotControlWithGlobalSpeeds(int robotId, float kickSpeed, float kickAngle, float dribblerSpeed, float xVelocity, float yVelocity,
                                                          float angularVelocity) {
    proto::sim::RobotCommand* command = this->controlCommand.add_robot_commands();
    command->set_id(robotId);
    command->set_kick_speed(kickSpeed);
    command->set_kick_angle(kickAngle);
    command->set_dribbler_speed(dribblerSpeed);

    proto::sim::MoveGlobalVelocity* velocity = command->mutable_move_command()->mutable_global_velocity();
    velocity->set_x(xVelocity);
    velocity->set_y(yVelocity);
    velocity->set_angular(angularVelocity);
}
proto::sim::RobotControl& RobotControlCommand::getPacket() { return this->controlCommand; }

FailedToBindPortException::FailedToBindPortException(const std::string message) { this->message = message; }

const char* FailedToBindPortException::what() const noexcept { return this->message.c_str(); }
}  // namespace rtt::robothub::simulation