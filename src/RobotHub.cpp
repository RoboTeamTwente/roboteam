#include <RobotCommand.h>
#include <RobotHub.h>

#include <cmath>
#include <iostream>
#include <sstream>

namespace rtt::robothub {

constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL = 30011;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL = 30012;
constexpr int DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION = 30013;

// These two values are properties of our physical robots. We use these in commands for simulators
constexpr float SIM_CHIPPER_ANGLE_DEGREES = 45.0f; // The angle at which the chipper shoots
constexpr float SIM_MAX_DRIBBLER_SPEED_RPM = 1021.0f; // The theoretical maximum speed of the dribblers

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
    this->basestationManager->setFeedbackCallback([&](const RobotFeedback &feedback, utils::TeamColor color) { this->handleRobotFeedbackFromBasestation(feedback, color); });
}

RobotHubStatistics& RobotHub::getStatistics() {
    this->statistics.basestationManagerStatus = this->basestationManager->getStatus();
    return this->statistics;
}

bool RobotHub::initializeNetworkers() {
    bool successfullyInitialized;

    try {
        this->robotCommandsBlueSubscriber = std::make_unique<rtt::net::RobotCommandsBlueSubscriber>([&](const proto::AICommand& commands) {
            this->onRobotCommands(commands, utils::TeamColor::BLUE);
        });

        this->robotCommandsYellowSubscriber = std::make_unique<rtt::net::RobotCommandsYellowSubscriber>([&](const proto::AICommand& commands) {
            this->onRobotCommands(commands, utils::TeamColor::YELLOW);
        });

        this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>([&](const proto::Setting& settings) {
            this->onSettings(settings);
        });

        this->simulationConfigurationSubscriber = std::make_unique<rtt::net::SimulationConfigurationSubscriber>([&](const proto::SimulationConfiguration& config){
            this->onSimulationConfiguration(config);
        });

        this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();

        successfullyInitialized = true;
    } catch (const std::exception& e) { // TODO: Figure out the exception
        successfullyInitialized = false;
    }

    return successfullyInitialized;
}

void RobotHub::sendCommandsToSimulator(const proto::AICommand &commands, utils::TeamColor color) {
    if (this->simulatorManager == nullptr) return;

    std::vector<int> robotIdsOfCommand; // Keeps track to which robots we will send a command

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : commands.commands()) {
        int id = robotCommand.id();
        float kickSpeed = robotCommand.chip_kick_vel();
        float kickAngle = robotCommand.chipper() ? SIM_CHIPPER_ANGLE_DEGREES : 0.0f;
        float dribblerSpeed = (robotCommand.dribbler() > 0 ? SIM_MAX_DRIBBLER_SPEED_RPM : 0.0);  // dribbler_speed is range of 0 to 1
        float xVelocity = robotCommand.vel().x();
        float yVelocity = robotCommand.vel().y();
        // TODO: Check if there is angular velocity
        float angularVelocity = robotCommand.w();

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        robotIdsOfCommand.push_back(id);

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

void RobotHub::sendCommandsToBasestation(const proto::AICommand &commands, utils::TeamColor color) {
    for (const proto::RobotCommand &protoCommand : commands.commands()) {
        // Convert the proto::RobotCommand to a RobotCommand for the basestation

        float rho = sqrtf(protoCommand.vel().x() * protoCommand.vel().x() + protoCommand.vel().y() * protoCommand.vel().y());
        float theta = atan2f(protoCommand.vel().y(), protoCommand.vel().x());
        auto bot = getWorldBot(protoCommand.id(), color, commands.extrapolatedworld());

        RobotCommand command;
        command.header = PACKET_TYPE_ROBOT_COMMAND;
        command.remVersion = LOCAL_REM_VERSION;
        command.id = protoCommand.id();

        command.doKick = protoCommand.kicker();
        command.doChip = protoCommand.chipper();
        command.doForce = protoCommand.chip_kick_forced();
        command.kickChipPower = protoCommand.chip_kick_vel();
        command.dribbler = (float)protoCommand.dribbler();

        command.rho = rho;
        command.theta = theta;

        command.angularControl = protoCommand.use_angle();
        command.angle = protoCommand.w();

        if (bot != nullptr) {
            command.useCameraAngle = true;
            command.cameraAngle = bot->angle();
        } else {
            command.useCameraAngle = false;
            command.cameraAngle = 0.0;
        }

        command.feedback = false;

        bool sentCommand = this->basestationManager->sendRobotCommand(command, color);

        // Update statistics
        this->statistics.incrementCommandsReceivedCounter(command.id, color);

        // Update bytes sent/packets dropped statistics
        if (sentCommand) {
            // TODO: Make this bytes instead of packets sent
            if (color == utils::TeamColor::YELLOW) {
                this->statistics.yellowTeamBytesSent++;
            } else {
                this->statistics.blueTeamBytesSent++;
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

void RobotHub::onRobotCommands(const proto::AICommand &commands, utils::TeamColor color) {
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
            std::cout << "[RobotHub]: Warning: Unknown robothub mode" << std::endl;
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
        const auto& ballLocation = configuration.ball_location();
        configCommand.setBallLocation(
            ballLocation.x(),
            ballLocation.y(),
            ballLocation.z(),
            ballLocation.x_velocity(),
            ballLocation.y_velocity(),
            ballLocation.z_velocity(),
            ballLocation.velocity_in_rolling(),
            ballLocation.teleport_safely(),
            ballLocation.by_force()
        );
    }

    for (const auto& robotLocation : configuration.robot_locations()) {
        configCommand.addRobotLocation(
            robotLocation.id(),
            robotLocation.is_team_yellow() ? utils::TeamColor::YELLOW : utils::TeamColor::BLUE,
            robotLocation.x(),
            robotLocation.y(),
            robotLocation.x_velocity(),
            robotLocation.y_velocity(),
            robotLocation.angular_velocity(),
            robotLocation.orientation(),
            robotLocation.present_on_field(),
            robotLocation.by_force()
        );
    }

    for (const auto& robotProperties : configuration.robot_properties()) {
        simulation::RobotProperties propertyValues = {
            .radius = robotProperties.radius(),
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
            .frontLeftWheelAngle = robotProperties.front_left_wheel_angle()
        };
        
        configCommand.addRobotSpecs(
            robotProperties.id(),
            robotProperties.is_team_yellow() ? utils::TeamColor::YELLOW : utils::TeamColor::BLUE,
            propertyValues
        );
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

void RobotHub::handleRobotFeedbackFromBasestation(const RobotFeedback &feedback, utils::TeamColor basestationColor) {
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

// TODO: Get rid of this function: It is crap!
// Copy of getWorldBot() because I don't want to pull in tactics as a
// dependency. If this function is moved to utils, we can use that
std::shared_ptr<proto::WorldRobot> getWorldBot(unsigned int id, utils::TeamColor color, const proto::World &world) {
    /** Heavily inefficient, copying over all the robots :(
     * If this was C++20 I would've picked std::span, but for now just use
     * yellow() / blue()
     */
    // if (ourTeam) {
    //     robots = std::vector<roboteam_proto::WorldRobot>(
    //     world.yellow().begin(),  world.yellow().end());
    // } else {
    //     robots =
    //     std::vector<roboteam_proto::WorldRobot>(world.blue().begin(),
    //     world.blue().end());
    // }

    // Prevent a copy.

    auto &robots = color == utils::TeamColor::YELLOW ? world.yellow() : world.blue();

    // https://en.cppreference.com/w/cpp/algorithm/find
    // Should do that instead, but whatever, doesn't really matter in terms of
    // performance
    for (const auto &bot : robots) {
        proto::WorldRobot a = bot;
        if (bot.id() == id) {
            return std::make_shared<proto::WorldRobot>(bot);
        }
    }
    return nullptr;
}

bool RobotHub::sendRobotFeedback(const proto::RobotData &feedback) { return this->robotFeedbackPublisher->publish(feedback); }

const char *FailedToInitializeNetworkersException::what() const throw() { return "Failed to initialize networker(s). Is another RobotHub running?"; }

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        auto& statistics = app.getStatistics();
        statistics.print();
        statistics.resetValues();
    }
}