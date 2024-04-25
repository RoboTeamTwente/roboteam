#include <REM_BaseTypes.h>
#include <REM_RobotCommand.h>
#include <RobotHub.h>
#include <RobotHubMode.h>
#include <roboteam_utils/Print.h>
#include <roboteam_utils/Time.h>
#include <roboteam_utils/Vector2.h>

#include <cmath>
#include <sstream>

namespace rtt::robothub {

constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL = 30011;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL = 30012;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION = 30013;

// These two values are properties of our physical robots. We use these in commands for simulators
constexpr float SIM_CHIPPER_ANGLE_DEGREES = 45.0f;     // The angle at which the chipper shoots
constexpr float SIM_MAX_DRIBBLER_SPEED_RPM = 1021.0f;  // The theoretical maximum speed of the dribblers

RobotHub::RobotHub(bool shouldLog, bool logInMarpleFormat) {
    simulation::SimulatorNetworkConfiguration config = {.blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
                                                        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
                                                        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION};

    if (!this->initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }
    {
        std::scoped_lock<std::mutex> lock(this->modeMutex);
        this->mode = rtt::net::RobotHubMode::UNKNOWN;
    }

    {
        std::scoped_lock<std::mutex> lock(this->simulatorManagerMutex);
        this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
        this->simulatorManager->setRobotControlFeedbackCallback([&](const simulation::RobotControlFeedback &feedback) { this->handleRobotFeedbackFromSimulator(feedback); });
    }
    {
        std::scoped_lock<std::mutex> lock(this->basestationManagerMutex);
        this->basestationManager = std::make_unique<basestation::BasestationManager>();
        this->basestationManager->setFeedbackCallback([&](const REM_RobotFeedback &feedback, rtt::Team color) { this->handleRobotFeedbackFromBasestation(feedback, color); });
        // this->basestationManager->setRobotStateInfoCallback([&](const REM_RobotStateInfo &robotStateInfo, rtt::Team color) { this->handleRobotStateInfo(robotStateInfo, color);
        // }); 
        // this->basestationManager->setBasestationLogCallback([&](const std::string &log, rtt::Team color) { this->handleBasestationLog(log, color); });
    }

    // if (shouldLog) { this->logger = RobotHubLogger(logInMarpleFormat); }
}

const RobotHubStatistics &RobotHub::getStatistics() {
    std::lock_guard<std::mutex> lock(basestationManagerMutex);
    this->statistics.basestationManagerStatus = this->basestationManager->getStatus();
    return this->statistics;
}

void RobotHub::resetStatistics() { this->statistics.resetValues(); }

bool RobotHub::initializeNetworkers() {
    bool successfullyInitialized;

    try {
        this->robotCommandsBlueSubscriber =
            std::make_unique<rtt::net::RobotCommandsBlueSubscriber>([&](const rtt::RobotCommands &commands) { this->onRobotCommands(commands, rtt::Team::BLUE); });

        this->robotCommandsYellowSubscriber =
            std::make_unique<rtt::net::RobotCommandsYellowSubscriber>([&](const rtt::RobotCommands &commands) { this->onRobotCommands(commands, rtt::Team::YELLOW); });

        this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>([&](const proto::GameSettings &_settings) { this->onSettings(_settings); });
        {
            std::lock_guard<std::mutex> lock(this->robotFeedbackPublisherMutex);
            this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();
        }

        successfullyInitialized = true;
    } catch (const std::exception &e) {  // TODO: Figure out the exception
        successfullyInitialized = false;
    }

    return successfullyInitialized;
}

void RobotHub::sendCommandsToSimulator(const rtt::RobotCommands &commands, rtt::Team color) {
    std::lock_guard<std::mutex> lock(simulatorManagerMutex);

    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (const auto &robotCommand : commands) {
        int id = robotCommand.id;
        auto kickSpeed = static_cast<float>(robotCommand.kickSpeed);
        float kickAngle = robotCommand.kickType == rtt::KickType::CHIP ? SIM_CHIPPER_ANGLE_DEGREES : 0.0f;
        float dribblerSpeed = static_cast<float>(robotCommand.dribblerSpeed) * SIM_MAX_DRIBBLER_SPEED_RPM;  // dribblerSpeed is range of 0 to 1
        auto angularVelocity = static_cast<float>(robotCommand.targetAngularVelocity);

        if (!robotCommand.useAngularVelocity) {
            RTT_WARNING("Robot command used absolute angle, but simulator requires angular velocity")
        }

        /* addRobotControlWithLocalSpeeds works with both grSim and ER-Force sim, while addRobotControlWithGlobalSpeeds only works with grSim*/
        auto relativeVelocity = robotCommand.velocity.rotate(-robotCommand.cameraAngleOfRobot);
        auto forward = relativeVelocity.x;
        auto left = relativeVelocity.y;
        simCommand.addRobotControlWithLocalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, forward, left, angularVelocity);

        // Update received commands stats
        this->statistics.incrementCommandsReceivedCounter(id, color);
    }

    auto bytesSent = this->simulatorManager->sendRobotControlCommand(simCommand, color);

    // Update bytes sent/packets dropped statistics
    {
        this->statistics.lockRobotStatsMutex();
        if (bytesSent > 0) {
            if (color == rtt::Team::YELLOW) {
                this->statistics.yellowTeamBytesSent += bytesSent;
            } else {
                this->statistics.blueTeamBytesSent += bytesSent;
            }
        } else {
            if (color == rtt::Team::YELLOW) {
                this->statistics.yellowTeamPacketsDropped++;
            } else {
                this->statistics.blueTeamPacketsDropped++;
            }
        }
        this->statistics.unlockRobotStatsMutex();
    }
}

void RobotHub::sendCommandsToBasestation(const rtt::RobotCommands &commands, rtt::Team color) {
    for (const auto &robotCommand : commands) {
        // Convert the RobotCommand to a command for the basestation

        REM_RobotCommand command = {};
        command.header = REM_PACKET_TYPE_REM_ROBOT_COMMAND;
        command.toRobotId = robotCommand.id;
        command.toColor = color == rtt::Team::BLUE;
        command.fromPC = true;
        command.remVersion = REM_LOCAL_VERSION;
        // command.messageId = 0; TODO implement incrementing message id
        command.payloadSize = REM_PACKET_SIZE_REM_ROBOT_COMMAND;
        command.timestamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count() / 10);

        command.kickAtAngle = robotCommand.kickAtAngle;
        command.doKick = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::KICK;
        command.doChip = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::CHIP;
        command.doForce = !robotCommand.waitForBall;
        command.kickChipPower = static_cast<float>(robotCommand.kickSpeed);
        command.dribbler = static_cast<float>(robotCommand.dribblerSpeed);

        command.rho = static_cast<float>(robotCommand.velocity.length());
        command.theta = -1.0f * static_cast<float>(robotCommand.velocity.angle());

        command.useAbsoluteAngle = !robotCommand.useAngularVelocity;
        command.angle = static_cast<float>(robotCommand.targetAngle.getValue());
        command.angularVelocity = static_cast<float>(robotCommand.targetAngularVelocity);

        command.useCameraAngle = robotCommand.cameraAngleOfRobotIsSet;
        command.cameraAngle = command.useCameraAngle ? static_cast<float>(robotCommand.cameraAngleOfRobot) : 0.0f;

        command.feedback = robotCommand.ignorePacket;

        // command.rho = 0;
        // command.theta = 0;
        // command.angularVelocity = 1;
        // command.useAbsoluteAngle = 0;

        int bytesSent = 0;
        {
            std::lock_guard<std::mutex> lock(basestationManagerMutex);
            bytesSent = this->basestationManager->sendRobotCommand(command, color);
        }

        // Update statistics
        this->statistics.incrementCommandsReceivedCounter(robotCommand.id, color);
        {
            this->statistics.lockRobotStatsMutex();

            if (bytesSent > 0) {
                if (color == rtt::Team::YELLOW) {
                    this->statistics.yellowTeamBytesSent += bytesSent;
                } else {
                    this->statistics.blueTeamBytesSent += bytesSent;
                }
            } else {
                if (color == rtt::Team::YELLOW) {
                    this->statistics.yellowTeamPacketsDropped++;
                } else {
                    this->statistics.blueTeamPacketsDropped++;
                }
            }
            this->statistics.unlockRobotStatsMutex();
        }
    }
}

void RobotHub::onRobotCommands(const rtt::RobotCommands &commands, rtt::Team color) {
    std::lock_guard<std::mutex> lock(this->onRobotCommandsMutex);
    std::lock_guard<std::mutex> lockMode(this->modeMutex);

    switch (this->mode) {
        case rtt::net::RobotHubMode::SIMULATOR:
            this->sendCommandsToSimulator(commands, color);
            break;
        case rtt::net::RobotHubMode::BASESTATION:
            this->sendCommandsToBasestation(commands, color);
            break;
        case rtt::net::RobotHubMode::UNKNOWN:
            // Do not handle commands
            break;
        default:
            RTT_WARNING("Unknown RobotHub mode")
            break;
    }

    // if (this->logger.has_value()) { this->logger.value().logRobotCommands(commands, color); }
}

void RobotHub::onSettings(const proto::GameSettings &_settings) {
    this->settings = _settings;
    rtt::net::RobotHubMode newMode = rtt::net::robotHubModeFromProto(_settings.robot_hub_mode());
    {
        std::lock_guard<std::mutex> lock(this->modeMutex);
        this->mode = newMode;
    }
    {
        std::lock_guard<std::mutex> lock(this->statistics.robotHubModeMutex);
        this->statistics.robotHubMode = newMode;
    }
}

void RobotHub::handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback) {
    rtt::RobotsFeedback robotsFeedback;
    robotsFeedback.source = rtt::RobotFeedbackSource::SIMULATOR;
    robotsFeedback.team = feedback.color;

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        rtt::RobotFeedback robotFeedback = {.id = robotId,
                                            .ballSensorSeesBall = hasBall,
                                            .ballPosition = 0,
                                            .ballSensorIsWorking = true,
                                            .dribblerSeesBall = hasBall,
                                            .velocity = {0, 0},
                                            .angle = 0,
                                            .xSensIsCalibrated = true,
                                            .capacitorIsCharged = true,
                                            .wheelLocked = 0,
                                            .wheelBraking = 0,
                                            .batteryLevel = 23.0f,
                                            .signalStrength = 0};
        robotsFeedback.feedback.push_back(robotFeedback);

        // Increment the feedback counter of this robot
        this->statistics.incrementFeedbackReceivedCounter(robotId, feedback.color);
    }

    this->sendRobotFeedback(robotsFeedback);
    this->handleSimulationErrors(feedback.simulationErrors);

    // if (this->logger.has_value()) { this->logger->logRobotFeedback(robotsFeedback); }
}

void RobotHub::handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, rtt::Team basestationColor) {
    rtt::RobotsFeedback robotsFeedback;
    robotsFeedback.source = rtt::RobotFeedbackSource::BASESTATION;
    robotsFeedback.team = basestationColor;

    rtt::RobotFeedback robotFeedback = {.id = static_cast<int>(feedback.fromRobotId),
                                        .ballSensorSeesBall = feedback.ballSensorSeesBall,
                                        .ballPosition = feedback.ballPos,
                                        .ballSensorIsWorking = feedback.ballSensorWorking,
                                        .dribblerSeesBall = feedback.dribblerSeesBall,
                                        .velocity = Vector2(Angle(feedback.theta), feedback.rho),
                                        .angle = Angle(feedback.angle),
                                        .xSensIsCalibrated = feedback.XsensCalibrated,
                                        .capacitorIsCharged = feedback.capacitorCharged,
                                        .wheelLocked = static_cast<int>(feedback.wheelLocked),
                                        .wheelBraking = static_cast<int>(feedback.wheelBraking),
                                        .batteryLevel = static_cast<float>(feedback.batteryLevel),
                                        .signalStrength = static_cast<int>(feedback.rssi)};
    robotsFeedback.feedback.push_back(robotFeedback);

    this->sendRobotFeedback(robotsFeedback);

    // Increment the feedback counter of this robot
    this->statistics.incrementFeedbackReceivedCounter(feedback.fromRobotId, basestationColor);

    // if (this->logger.has_value()) { this->logger->logRobotFeedback(robotsFeedback); }
}

bool RobotHub::sendRobotFeedback(const rtt::RobotsFeedback &feedback) {
    this->statistics.lockRobotStatsMutex();
    std::size_t bytesSent = 0;
    {
        std::lock_guard<std::mutex> lock(this->robotFeedbackPublisherMutex);
        bytesSent = this->robotFeedbackPublisher->publish(feedback);
    }
    this->statistics.feedbackBytesSent += bytesSent;
    this->statistics.unlockRobotStatsMutex();
    return bytesSent > 0;
}

// void RobotHub::handleRobotStateInfo(const REM_RobotStateInfo &info, rtt::Team team) {
//    // if (this->logger.has_value()) { this->logger->logRobotStateInfo(info, team); }
// }

// void RobotHub::handleBasestationLog(const std::string &basestationLogMessage, rtt::Team team) {
//     // if (this->logger.has_value()) { this->logger->logInfo("[" + teamToString(team) + "] " + basestationLogMessage); }
//     // RTT_INFO("Basestation ", teamToString(team), ": ", basestationLogMessage)
// }

void RobotHub::handleSimulationErrors(const std::vector<simulation::SimulationError> &errors) {
    for (const auto &error : errors) {
        std::string message;
        if (error.code.has_value() && error.message.has_value())
            message = "Received Simulation error! code=" + error.code.value() + "    message=" + error.message.value();
        else if (error.code.has_value())
            message = "Received Simulation error! code=" + error.code.value();
        else if (error.message.has_value())
            message = "Received Simulation error! message=" + error.message.value();
        else
            message = "Received unknown Simulation error";

        RTT_ERROR(message);
        // if (this->logger.has_value()) { this->logger->logInfo("WARNING: " + message); }
    }
}

const char *FailedToInitializeNetworkersException::what() const noexcept { return "Failed to initialize networker(s). Is another RobotHub running?"; }

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    // auto itLog = std::find(argv, argv + argc, std::string("-log"));
    // bool shouldLog = itLog != argv + argc;

    // auto itMarple = std::find(argv, argv + argc, std::string("-marple"));
    // bool logForMarple = itMarple != argv + argc;

    // if (logForMarple) shouldLog = true; // Log for marple means to log

    // rtt::robothub::RobotHub app(shouldLog, logForMarple);
    rtt::robothub::RobotHub app(false, false);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        {
            app.getStatistics().print();
            app.resetStatistics();
        }
    }
}