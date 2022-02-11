#include <roboteam_utils/Print.h>

#include <simulation/SimulatorManager.hpp>

namespace rtt::robothub::simulation {

constexpr int LISTEN_THREAD_COOLDOWN_MS = 10;  // Small cooldown in thread between checking for new messsages

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

    RTT_INFO(
        "\n"
        "SimulationManager bound on:",
        "\n", " - Blue Control Port: ", config.blueControlPort, "\n", " - Blue Feedback Port: ", config.blueFeedbackPort, "\n",
        " - Yellow Control Port: ", config.yellowControlPort, "\n", " - Yellow Feedback Port: ", config.yellowFeedbackPort, "\n",
        " - Simulation Control Port: ", config.configurationPort, "\n", " - Simulation Feedback Port: ", config.configurationFeedbackPort)

    // Spawn listening threads that handle incoming feedback
    this->blueFeedbackListenThread = std::thread([this] { listenForRobotControlFeedback(utils::TeamColor::BLUE); });
    this->yellowFeedbackListenThread = std::thread([this] { listenForRobotControlFeedback(utils::TeamColor::YELLOW); });
    this->configurationFeedbackListenThread = std::thread([this] { listenForConfigurationFeedback(); });
}

SimulatorManager::~SimulatorManager() { this->stopFeedbackListeningThreads(); }

int SimulatorManager::sendRobotControlCommand(RobotControlCommand& robotControlCommand, utils::TeamColor color) {
    int bytesSent;

    switch (color) {
        case utils::TeamColor::YELLOW: {
            bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->yellowControlSocket, this->networkConfiguration.yellowControlPort);
            break;
        }
        case utils::TeamColor::BLUE: {
            bytesSent = this->sendPacket(robotControlCommand.getPacket(), this->blueControlSocket, this->networkConfiguration.blueControlPort);
            break;
        }
        default: {
            bytesSent = 0;
        }
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

void SimulatorManager::listenForRobotControlFeedback(utils::TeamColor color) {
    // Select the socket that is used by the team
    QUdpSocket& teamSocket = (color == utils::TeamColor::YELLOW) ? this->yellowControlSocket : this->blueControlSocket;

    while (!this->shouldStopListeningToFeedback) {
        while (teamSocket.hasPendingDatagrams()) {
            QNetworkDatagram datagram = teamSocket.receiveDatagram();
            if (datagram.isValid()) {
                RobotControlFeedback f = this->getControlFeedbackFromDatagram(datagram, color);
                this->callRobotControlFeedbackCallback(f);
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(LISTEN_THREAD_COOLDOWN_MS));
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
        std::this_thread::sleep_for(std::chrono::milliseconds(LISTEN_THREAD_COOLDOWN_MS));
    }
}

RobotControlFeedback SimulatorManager::getControlFeedbackFromDatagram(QNetworkDatagram& datagram, utils::TeamColor color) {
    // First, parse datagram into the proto packet that it resembles
    proto::simulation::RobotControlResponse response;
    response.ParseFromArray(datagram.data().data(), datagram.data().size());

    // Then, put the information in an easy accessible struct
    RobotControlFeedback feedback;
    feedback.color = color;

    for (const proto::simulation::RobotFeedback& robotFeedback : response.feedback()) {
        // The dribbler_ball_contact boolean is optional, so both check if it has been set and check if it is set to true
        bool robotHasBall = robotFeedback.has_dribbler_ball_contact() && robotFeedback.dribbler_ball_contact();
        feedback.robotIdHasBall[robotFeedback.id()] = robotHasBall;
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