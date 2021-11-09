// By: Floris Heinen
#include "simulation/SimulatorManager.hpp"

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

    std::cout << "SimulationManager bound on: " << std::endl 
        << " - Blue Control Port: " << config.blueControlPort << std::endl
        << " - Blue Feedback Port: " << config.blueFeedbackPort << std::endl
        << " - Yellow Control Port: " << config.yellowControlPort << std::endl
        << " - Yellow Feedback Port: " << config.yellowFeedbackPort << std::endl
        << " - Simulation Control Port: " << config.configurationPort << std::endl
        << " - Simulation Feedback Port: " << config.configurationFeedbackPort << std::endl;

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
                this->callConfigurationFeedbackCallback(f);
            }
        }
    }
}

RobotControlFeedback SimulatorManager::getControlFeedbackFromDatagram(QNetworkDatagram& datagram, bool isTeamYellow) {
    // First, parse datagram into the proto packet that it resembles
    proto::simulation::RobotControlResponse response;
    response.ParseFromArray(datagram.data().data(), datagram.data().size());

    // Then, put the information in an easy accessible struct
    RobotControlFeedback feedback;
    feedback.isTeamYellow = isTeamYellow;

    for (const proto::simulation::RobotFeedback& robotFeedback : response.feedback()) {
        // This boolean is optional, so first check if it has been set before checking if it is set to true
        if (robotFeedback.has_dribbler_ball_contact() && robotFeedback.dribbler_ball_contact()) feedback.robotsThatHaveTheBall.push_back(robotFeedback.id());
    }

    for (const proto::simulation::SimulatorError& simulatorError : response.errors()) {
        SimulationError error;
        if (simulatorError.has_code()) error.code = simulatorError.code();
        if (simulatorError.has_message()) error.message = simulatorError.message();
        feedback.simulationErrors.push_back(error);
    }

    return feedback;
}

ConfigurationFeedback SimulatorManager::getConfigurationFeedbackFromDatagram(QNetworkDatagram& datagram) {
    // First, parse datagram into the proto packet that it resembles
    proto::simulation::SimulatorResponse response;
    response.ParseFromArray(datagram.data().data(), datagram.data().size());

    // Then, put the information in an easy accesible struct
    ConfigurationFeedback feedback;
    for (const proto::simulation::SimulatorError& simulationError : response.errors()) {
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

FailedToBindPortException::FailedToBindPortException(const std::string message) { this->message = message; }

const char* FailedToBindPortException::what() const noexcept { return this->message.c_str(); }
}  // namespace rtt::robothub::simulation