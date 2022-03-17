#include <REM_RobotCommand.h>
#include <RobotHub.h>
#include <roboteam_utils/Print.h>

#include <cmath>
#include <sstream>

namespace rtt::robothub {

constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL = 30011;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL = 30012;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION = 30013;

// These two values are properties of our physical robots. We use these in commands for simulators
constexpr float SIM_CHIPPER_ANGLE_DEGREES = 45.0f;     // The angle at which the chipper shoots
constexpr float SIM_MAX_DRIBBLER_SPEED_RPM = 1021.0f;  // The theoretical maximum speed of the dribblers

RobotHub::RobotHub() {
    simulation::SimulatorNetworkConfiguration config = {.blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
                                                        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
                                                        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION};

    if (!this->initializeNetworkers()) {
        throw FailedToInitializeNetworkersException();
    }

    this->mode = utils::RobotHubMode::NEITHER;

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    this->simulatorManager->setRobotControlFeedbackCallback([&](const simulation::RobotControlFeedback &feedback) { this->handleRobotFeedbackFromSimulator(feedback); });

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    this->basestationManager->setFeedbackCallback([&](const REM_RobotFeedback &feedback, utils::TeamColor color) { this->handleRobotFeedbackFromBasestation(feedback, color); });
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
            std::make_unique<rtt::net::RobotCommandsBlueSubscriber>([&](const rtt::RobotCommands &commands) { this->onRobotCommands(commands, utils::TeamColor::BLUE); });

        this->robotCommandsYellowSubscriber =
            std::make_unique<rtt::net::RobotCommandsYellowSubscriber>([&](const rtt::RobotCommands &commands) { this->onRobotCommands(commands, utils::TeamColor::YELLOW); });

        this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>([&](const proto::Setting &settings) { this->onSettings(settings); });

        this->simulationConfigurationSubscriber =
            std::make_unique<rtt::net::SimulationConfigurationSubscriber>([&](const proto::SimulationConfiguration &config) { this->onSimulationConfiguration(config); });

        this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();

        successfullyInitialized = true;
    } catch (const std::exception &e) {  // TODO: Figure out the exception
        successfullyInitialized = false;
    }

    return successfullyInitialized;
}

void RobotHub::sendCommandsToSimulator(const rtt::RobotCommands &commands, utils::TeamColor color) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (const auto &robotCommand : commands) {
        int id = robotCommand.id;
        float kickSpeed = static_cast<float>(robotCommand.kickSpeed);
        float kickAngle = robotCommand.kickType == rtt::KickType::CHIP ? SIM_CHIPPER_ANGLE_DEGREES : 0.0f;
        float dribblerSpeed = static_cast<float>(robotCommand.dribblerSpeed) * SIM_MAX_DRIBBLER_SPEED_RPM;  // dribblerSpeed is range of 0 to 1
        float xVelocity = static_cast<float>(robotCommand.velocity.x);
        float yVelocity = static_cast<float>(robotCommand.velocity.y);
        float angularVelocity = static_cast<float>(robotCommand.targetAngularVelocity);

        if (!robotCommand.useAngularVelocity) {
            RTT_WARNING("Robot command used absolute angle, but simulator requires angular velocity")
        }

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update received commands stats
        this->statistics.incrementCommandsReceivedCounter(id, color);
    }

    int bytesSent = this->simulatorManager->sendRobotControlCommand(simCommand, color);

    // Update bytes sent/packets dropped statistics
    if (bytesSent > 0) {
        if (color == utils::TeamColor::YELLOW) {
            this->statistics.yellowTeamBytesSent += bytesSent;
        } else {
            this->statistics.blueTeamBytesSent += bytesSent;
        }
    } else {
        if (color == utils::TeamColor::YELLOW) {
            this->statistics.yellowTeamPacketsDropped++;
        } else {
            this->statistics.blueTeamPacketsDropped++;
        }
    }
}

void RobotHub::sendCommandsToBasestation(const rtt::RobotCommands &commands, utils::TeamColor color) {
    for (const auto &robotCommand : commands) {
        // Convert the RobotCommand to a command for the basestation

        REM_RobotCommand command;
        command.header = PACKET_TYPE_REM_ROBOT_COMMAND;
        command.remVersion = LOCAL_REM_VERSION;
        command.id = robotCommand.id;

        command.doKick = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::NORMAL;
        command.doChip = robotCommand.kickSpeed > 0.0 && robotCommand.kickType == KickType::CHIP;
        command.doForce = !robotCommand.waitForBall;
        command.kickChipPower = static_cast<float>(robotCommand.kickSpeed);
        command.dribbler = static_cast<float>(robotCommand.dribblerSpeed);

        command.rho = static_cast<float>(robotCommand.velocity.length());
        command.theta = static_cast<float>(robotCommand.velocity.angle());

        command.angularControl = !robotCommand.useAngularVelocity;
        command.angle = robotCommand.useAngularVelocity ? static_cast<float>(robotCommand.targetAngularVelocity) : static_cast<float>(robotCommand.targetAngle.getValue());
        if (robotCommand.useAngularVelocity) {
            RTT_WARNING("Robot command used angular velocity, but robots do not support that yet")
        }

        if (robotCommand.cameraAngleOfRobotIsSet) {
            command.cameraAngle = static_cast<float>(robotCommand.cameraAngleOfRobot);
        } else {
            command.cameraAngle = 0.0f;
        }

        command.feedback = robotCommand.ignorePacket;

        int bytesSent = this->basestationManager->sendRobotCommand(command, color);

        // Update statistics
        this->statistics.incrementCommandsReceivedCounter(robotCommand.id, color);

        if (bytesSent > 0) {
            if (color == utils::TeamColor::YELLOW) {
                this->statistics.yellowTeamBytesSent += bytesSent;
            } else {
                this->statistics.blueTeamBytesSent += bytesSent;
            }
        } else {
            if (color == utils::TeamColor::YELLOW) {
                this->statistics.yellowTeamPacketsDropped++;
            } else {
                this->statistics.blueTeamPacketsDropped++;
            }
        }
    }
}

void RobotHub::onRobotCommands(const rtt::RobotCommands &commands, utils::TeamColor color) {
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
}

void RobotHub::onSettings(const proto::Setting &settings) {
    this->settings = settings;

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
        configCommand.addRobotLocation(robotLocation.id(), robotLocation.is_team_yellow() ? utils::TeamColor::YELLOW : utils::TeamColor::BLUE, robotLocation.x(), robotLocation.y(),
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

        configCommand.addRobotSpecs(robotProperties.id(), robotProperties.is_team_yellow() ? utils::TeamColor::YELLOW : utils::TeamColor::BLUE, propertyValues);
    }

    int bytesSent = this->simulatorManager->sendConfigurationCommand(configCommand);
    // TODO: Put these bytes sent into nice statistics output (low priority)
}

void RobotHub::handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback) {
    proto::RobotData feedbackToBePublished;
    feedbackToBePublished.set_isyellow(feedback.color == utils::TeamColor::YELLOW);

    // proto::RobotFeedback* feedbackOfRobots = feedbackToBePublished.mutable_receivedfeedback();

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
        feedbackOfRobot->set_id(robotId);
        feedbackOfRobot->set_hasball(hasBall);

        // Increment the feedback counter of this robot
        this->statistics.incrementFeedbackReceivedCounter(robotId, feedback.color);
    }

    this->sendRobotFeedback(feedbackToBePublished);
}

void RobotHub::handleRobotFeedbackFromBasestation(const REM_RobotFeedback &feedback, utils::TeamColor basestationColor) {
    proto::RobotData feedbackToBePublished;
    feedbackToBePublished.set_isyellow(basestationColor == utils::TeamColor::YELLOW);

    proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
    feedbackOfRobot->set_id(feedback.id);
    feedbackOfRobot->set_xsenscalibrated(feedback.XsensCalibrated);
    feedbackOfRobot->set_ballsensorisworking(feedback.ballSensorWorking);
    feedbackOfRobot->set_hasball(feedback.hasBall);
    feedbackOfRobot->set_ballpos(feedback.ballPos);
    // feedbackOfRobot->set_capacitorcharged(feedback.capacitorCharged); // Not yet added to proto
    feedbackOfRobot->set_x_vel(std::cos(feedback.theta) * feedback.rho);
    feedbackOfRobot->set_y_vel(std::sin(feedback.theta) * feedback.rho);
    feedbackOfRobot->set_yaw(feedback.angle);
    // feedbackOfRobot->set_batterylevel(feedback.batteryLevel); // Not in proto yet
    feedbackOfRobot->set_signalstrength(feedback.rssi);
    feedbackOfRobot->set_haslockedwheel(feedback.wheelLocked);

    this->sendRobotFeedback(feedbackToBePublished);

    // Increment the feedback counter of this robot
    this->statistics.incrementFeedbackReceivedCounter(feedback.id, basestationColor);
}

bool RobotHub::sendRobotFeedback(const proto::RobotData &feedback) {
    this->statistics.feedbackBytesSent += static_cast<int>(feedback.ByteSizeLong());
    return this->robotFeedbackPublisher->publish(feedback);
}

const char *FailedToInitializeNetworkersException::what() const throw() { return "Failed to initialize networker(s). Is another RobotHub running?"; }

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.getStatistics().print();
        app.resetStatistics();
    }
}