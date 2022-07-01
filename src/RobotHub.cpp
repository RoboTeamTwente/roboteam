#include <REM_RobotCommand.h>
#include <RobotHub.h>
#include <roboteam_utils/Print.h>
#include <roboteam_utils/Time.h>


#include <cmath>

#include <sstream>
#include <roboteam_utils/Vector2.h>

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

    this->mode = utils::RobotHubMode::NEITHER;

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    this->simulatorManager->setRobotControlFeedbackCallback([&](const simulation::RobotControlFeedback &feedback) { this->handleRobotFeedbackFromSimulator(feedback); });
    this->simulatorManager->setConfigurationFeedbackCallback([&](const simulation::ConfigurationFeedback &feedback) { this->handleSimulationConfigurationFeedback(feedback); });

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    this->basestationManager->setFeedbackCallback([&](const REM_RobotFeedback &feedback, rtt::Team color) { this->handleRobotFeedbackFromBasestation(feedback, color); });
    this->basestationManager->setRobotStateInfoCallback([&](const REM_RobotStateInfo& robotStateInfo, rtt::Team color) { this->handleRobotStateInfo(robotStateInfo, color); });
    this->basestationManager->setBasestationLogCallback([&](const std::string& log, rtt::Team color) { this->handleBasestationLog(log, color); });

    if (shouldLog) { this->logger = RobotHubLogger(logInMarpleFormat); }
}

const RobotHubStatistics &RobotHub::getStatistics() {
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

        this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>([&](const proto::Setting &_settings) { this->onSettings(_settings); });

        this->simulationConfigurationSubscriber =
            std::make_unique<rtt::net::SimulationConfigurationSubscriber>([&](const proto::SimulationConfiguration &config) { this->onSimulationConfiguration(config); });

        this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();

        successfullyInitialized = true;
    } catch (const std::exception &e) {  // TODO: Figure out the exception
        successfullyInitialized = false;
    }

    return successfullyInitialized;
}

void RobotHub::sendCommandsToSimulator(const rtt::RobotCommands &commands, rtt::Team color) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (const auto &robotCommand : commands) {
        int id = robotCommand.id;
        auto kickSpeed = static_cast<float>(robotCommand.kickSpeed);
        float kickAngle = robotCommand.kickType == rtt::KickType::CHIP ? SIM_CHIPPER_ANGLE_DEGREES : 0.0f;
        float dribblerSpeed = static_cast<float>(robotCommand.dribblerSpeed) * SIM_MAX_DRIBBLER_SPEED_RPM;  // dribblerSpeed is range of 0 to 1
        auto xVelocity = static_cast<float>(robotCommand.velocity.x);
        auto yVelocity = static_cast<float>(robotCommand.velocity.y);
        auto angularVelocity = static_cast<float>(robotCommand.targetAngularVelocity);

        if (!robotCommand.useAngularVelocity) {
            RTT_WARNING("Robot command used absolute angle, but simulator requires angular velocity")
        }

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update received commands stats
        this->statistics.incrementCommandsReceivedCounter(id, color);
    }

    auto bytesSent = this->simulatorManager->sendRobotControlCommand(simCommand, color);

    // Update bytes sent/packets dropped statistics
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
}

void RobotHub::sendCommandsToBasestation(const rtt::RobotCommands &commands, rtt::Team color) {
    for (const auto &robotCommand : commands) {
        // Convert the RobotCommand to a command for the basestation

        REM_RobotCommand command;
        command.header = PACKET_TYPE_REM_ROBOT_COMMAND;
        command.remVersion = LOCAL_REM_VERSION;
        command.id = robotCommand.id;

        command.doKick = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::KICK;
        command.doChip = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::CHIP;
        command.doForce = !robotCommand.waitForBall;
        command.kickChipPower = static_cast<float>(robotCommand.kickSpeed);
        command.dribbler = static_cast<float>(robotCommand.dribblerSpeed);

        command.rho = static_cast<float>(robotCommand.velocity.length());
        command.theta = static_cast<float>(robotCommand.velocity.angle());

        command.useAbsoluteAngle = !robotCommand.useAngularVelocity;
        command.angle = static_cast<float>(robotCommand.targetAngle.getValue());
        command.angularVelocity = static_cast<float>(robotCommand.targetAngularVelocity);

        command.useCameraAngle = robotCommand.cameraAngleOfRobotIsSet;
        command.cameraAngle = command.useCameraAngle ? static_cast<float>(robotCommand.cameraAngleOfRobot) : 0.0f;

        command.feedback = robotCommand.ignorePacket;

        int bytesSent = this->basestationManager->sendRobotCommand(command, color);

        // Update statistics
        this->statistics.incrementCommandsReceivedCounter(robotCommand.id, color);

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
    }
}

void RobotHub::onRobotCommands(const rtt::RobotCommands &commands, rtt::Team color) {
    std::scoped_lock<std::mutex> lock(this->onRobotCommandsMutex);

    switch (this->mode) {
        case utils::RobotHubMode::SIMULATOR:
            this->sendCommandsToSimulator(commands, color);
            break;
        case utils::RobotHubMode::BASESTATION:
            this->sendCommandsToBasestation(commands, color);
            break;
        case utils::RobotHubMode::NEITHER:
            // Do not handle commands
            break;
        default:
            RTT_WARNING("Unknown RobotHub mode")
            break;
    }

    if (this->logger.has_value()) { this->logger.value().logRobotCommands(commands, color); }
}

void RobotHub::onSettings(const proto::Setting &_settings) {
    this->settings = _settings;

    utils::RobotHubMode newMode = settings.serialmode() ? utils::RobotHubMode::BASESTATION : utils::RobotHubMode::SIMULATOR;

    this->mode = newMode;
    this->statistics.robotHubMode = newMode;
}

void RobotHub::onSimulationConfiguration(const proto::SimulationConfiguration &configuration) {
    simulation::ConfigurationCommand configCommand;

    if (configuration.has_ball_location()) {
        const auto &ballLocation = configuration.ball_location();
        configCommand.setBallLocation(ballLocation.x(), ballLocation.y(), ballLocation.z(), ballLocation.x_velocity(), ballLocation.y_velocity(), ballLocation.z_velocity(),
                                      ballLocation.velocity_in_rolling(), ballLocation.teleport_safely(), ballLocation.by_force());
    }

    for (const auto &robotLocation : configuration.robot_locations()) {
        configCommand.addRobotLocation(robotLocation.id(), robotLocation.is_team_yellow() ? rtt::Team::YELLOW : rtt::Team::BLUE, robotLocation.x(), robotLocation.y(),
                                       robotLocation.x_velocity(), robotLocation.y_velocity(), robotLocation.angular_velocity(), robotLocation.orientation(),
                                       robotLocation.present_on_field(), robotLocation.by_force());
    }

    for (const auto &robotProperties : configuration.robot_properties()) {
        simulation::RobotProperties propertyValues = {.radius = robotProperties.radius(),
                                                      .height = robotProperties.height(),
                                                      .mass = robotProperties.mass(),
                                                      .maxKickSpeed = robotProperties.max_kick_speed(),
                                                      .maxChipSpeed = robotProperties.max_chip_speed(),
                                                      .centerToDribblerDistance = robotProperties.center_to_dribbler_distance(),
                                                      // Movement limits
                                                      .maxAcceleration = robotProperties.max_acceleration(),
                                                      .maxAngularAcceleration = robotProperties.max_angular_acceleration(),
                                                      .maxDeceleration = robotProperties.max_deceleration(),
                                                      .maxAngularDeceleration = robotProperties.max_angular_deceleration(),
                                                      .maxVelocity = robotProperties.max_velocity(),
                                                      .maxAngularVelocity = robotProperties.max_angular_velocity(),
                                                      // Wheel angles
                                                      .frontRightWheelAngle = robotProperties.front_right_wheel_angle(),
                                                      .backRightWheelAngle = robotProperties.back_right_wheel_angle(),
                                                      .backLeftWheelAngle = robotProperties.back_left_wheel_angle(),
                                                      .frontLeftWheelAngle = robotProperties.front_left_wheel_angle()};

        configCommand.addRobotSpecs(robotProperties.id(), robotProperties.is_team_yellow() ? rtt::Team::YELLOW : rtt::Team::BLUE, propertyValues);
    }

    // TODO: Put these bytes sent into nice statistics output (low priority)
    this->simulatorManager->sendConfigurationCommand(configCommand);
}

void RobotHub::handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback) {
    rtt::RobotsFeedback robotsFeedback;
    robotsFeedback.source = rtt::RobotFeedbackSource::SIMULATOR;
    robotsFeedback.team = feedback.color;

    for (auto const&[robotId, hasBall] : feedback.robotIdHasBall) {
        rtt::RobotFeedback robotFeedback = {
            .id = robotId,
            .hasBall = hasBall,
            .ballPosition = 0,
            .ballSensorIsWorking = true,
            .velocity = { 0, 0 },
            .angle = 0,
            .xSensIsCalibrated = true,
            .capacitorIsCharged = true,
            .wheelLocked = 0,
            .wheelBraking = 0,
            .batteryLevel = 23.0f,
            .signalStrength = 0
        };
        robotsFeedback.feedback.push_back(robotFeedback);

        // Increment the feedback counter of this robot
        this->statistics.incrementFeedbackReceivedCounter(robotId, feedback.color);
    }

    this->sendRobotFeedback(robotsFeedback);
    this->handleSimulationErrors(feedback.simulationErrors);

    if (this->logger.has_value()) { this->logger->logRobotFeedback(robotsFeedback); }
}

void RobotHub::handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, rtt::Team basestationColor) {
    rtt::RobotsFeedback robotsFeedback;
    robotsFeedback.source = rtt::RobotFeedbackSource::BASESTATION;
    robotsFeedback.team = basestationColor;

    rtt::RobotFeedback robotFeedback = {
        .id = static_cast<int>(feedback.id),
        .hasBall = feedback.ballSensorSeesBall,
        .ballPosition = feedback.ballPos,
        .ballSensorIsWorking = feedback.ballSensorWorking,
        // .dribblerSeesBall = feedback.dribblerSeesBall // TODO: Implement this on the AI side
        .velocity = Vector2(Angle(feedback.theta), feedback.rho),
        .angle = Angle(feedback.angle),
        .xSensIsCalibrated = feedback.XsensCalibrated,
        .capacitorIsCharged = feedback.capacitorCharged,
        .wheelLocked = static_cast<int>(feedback.wheelLocked),
        .wheelBraking = static_cast<int>(feedback.wheelBraking),
        .batteryLevel = static_cast<float>(feedback.batteryLevel),
        .signalStrength = static_cast<int>(feedback.rssi)
    };
    robotsFeedback.feedback.push_back(robotFeedback);

    this->sendRobotFeedback(robotsFeedback);

    // Increment the feedback counter of this robot
    this->statistics.incrementFeedbackReceivedCounter(feedback.id, basestationColor);

    if (this->logger.has_value()) { this->logger->logRobotFeedback(robotsFeedback); }
}

bool RobotHub::sendRobotFeedback(const rtt::RobotsFeedback &feedback) {
    auto bytesSent = this->robotFeedbackPublisher->publish(feedback);
    this->statistics.feedbackBytesSent += bytesSent;
    return bytesSent > 0;
}

void RobotHub::handleSimulationConfigurationFeedback(const simulation::ConfigurationFeedback &configFeedback) {
    this->handleSimulationErrors(configFeedback.simulationErrors);
}

void RobotHub::handleRobotStateInfo(const REM_RobotStateInfo& info, rtt::Team team) {
    if (this->logger.has_value()) { this->logger->logRobotStateInfo(info, team); }
}

void RobotHub::handleBasestationLog(const std::string &basestationLogMessage, rtt::Team team) {
    if (this->logger.has_value()) { this->logger->logInfo("[" + teamToString(team) + "] " + basestationLogMessage); }
    RTT_DEBUG("Basestation ", teamToString(team), ": ", basestationLogMessage)
}

void RobotHub::handleSimulationErrors(const std::vector<simulation::SimulationError> &errors) {
    for (const auto& error : errors) {
        std::string message;
        if (error.code.has_value() && error.message.has_value())
            message = "Received Simulation error ", error.code.value(), ": ", error.message.value();
        else if (error.code.has_value())
            message = "Received Simulation error with code: ", error.code.value();
        else if (error.message.has_value())
            message = "Received Simulation error: ", error.message.value();
        else
            message = "Received unknown Simulation error";

        RTT_ERROR(message);
        if (this->logger.has_value()) { this->logger->logInfo("WARNING: " + message); }
    }
}

const char *FailedToInitializeNetworkersException::what() const noexcept { return "Failed to initialize networker(s). Is another RobotHub running?"; }

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    auto itLog = std::find(argv, argv + argc, std::string("-log"));
    bool shouldLog = itLog != argv + argc;

    auto itMarple = std::find(argv, argv + argc, std::string("-marple"));
    bool logForMarple = itMarple != argv + argc;

    if (logForMarple) shouldLog = true; // Log for marple means to log

    rtt::robothub::RobotHub app(shouldLog, logForMarple);

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.getStatistics().print();
        app.resetStatistics();
    }
}