#include <RobotHub.h>

#include <cmath>
#include <iostream>
#include <sstream>

#include <RobotCommand.h>

namespace rtt::robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    simulation::SimulatorNetworkConfiguration config = {.blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
                                                        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
                                                        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION};

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    auto simulationFeedbackCallback = std::bind(&RobotHub::handleRobotFeedbackFromSimulator, this, std::placeholders::_1);
    this->simulatorManager->setRobotControlFeedbackCallback(simulationFeedbackCallback);

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    auto basestationFeedbackCallback = std::bind(&RobotHub::handleRobotFeedbackFromBasestation, this, std::placeholders::_1);
    this->basestationManager->setFeedbackCallback(basestationFeedbackCallback);

    this->subscribe();
}

void RobotHub::subscribe() {
    auto blueCommandsCallback = std::bind(&RobotHub::onBlueRobotCommands, this, std::placeholders::_1);
    this->robotCommandsBlueSubscriber = std::make_unique<rtt::net::RobotCommandsBlueSubscriber>(blueCommandsCallback);
    
    auto yellowCommandsCallback = std::bind(&RobotHub::onYellowRobotCommands, this, std::placeholders::_1);
    this->robotCommandsYellowSubscriber = std::make_unique<rtt::net::RobotCommandsYellowSubscriber>(yellowCommandsCallback);

    auto settingsCallback = std::bind(&RobotHub::onSettingsFromChannel1, this, std::placeholders::_1);
    this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>(settingsCallback);

    this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();
}

void RobotHub::sendCommandsToSimulator(const proto::RobotCommands &commands, bool toTeamYellow) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : commands.commands()) {
        int id = robotCommand.id();
        float kickSpeed = robotCommand.kick_speed();
        float kickAngle = robotCommand.use_chipper() ? DEFAULT_CHIPPER_ANGLE : 0.0f;
        float dribblerSpeed = robotCommand.dribbler_speed() * MAX_DRIBBLER_SPEED; // dribbler_speed is range of 0 to 1
        float xVelocity = robotCommand.x_velocity();
        float yVelocity = robotCommand.y_velocity();
        // TODO: Check if there is angular velocity
        float angularVelocity = robotCommand.rotation();

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update statistics
        this->commands_sent[id]++;
    }

    this->simulatorManager->sendRobotControlCommand(simCommand, toTeamYellow);
}

void RobotHub::sendCommandsToBasestation(const proto::RobotCommands &commands, bool toTeamYellow) {
    for (const proto::RobotCommand &protoCommand : commands.commands()) {
        // Convert the proto::RobotCommand to a RobotCommand for the basestation

        float rho = sqrtf(protoCommand.x_velocity() * protoCommand.x_velocity() + protoCommand.y_velocity() * protoCommand.y_velocity());
        float theta = atan2f(protoCommand.y_velocity(), protoCommand.x_velocity());
        auto bot = rtt::robothub::utils::getWorldBot(protoCommand.id(), toTeamYellow, world);

        RobotCommand command;
        command.header = PACKET_TYPE_ROBOT_COMMAND;
        command.remVersion = LOCAL_REM_VERSION;
        command.id = protoCommand.id();

        command.doKick = !protoCommand.use_chipper() && (protoCommand.kick_speed() > 0);
        command.doChip =  protoCommand.use_chipper() && (protoCommand.kick_speed() > 0);
        command.doForce = protoCommand.dont_wait_for_ball();
        command.kickChipPower = protoCommand.kick_speed();
        command.dribbler = protoCommand.dribbler_speed();

        command.rho = rho;
        command.theta = theta;

        command.angularControl = protoCommand.rotation_as_velocity();
        command.angle = protoCommand.rotation();

        if (bot != nullptr) {
            command.useCameraAngle = true;
            command.cameraAngle = bot->rotation();
        } else {
            command.useCameraAngle = false;
            command.cameraAngle = 0.0;
        }
        
        command.feedback = false;

        this->basestationManager->sendSerialCommand(command);

        // Update statistics
        commands_sent[protoCommand.id()]++;
    }
}

void RobotHub::onBlueRobotCommands(const proto::RobotCommands &commands) {
    this->processRobotCommands(commands, false, this->mode);
}
void RobotHub::onYellowRobotCommands(const proto::RobotCommands &commands) {
    this->processRobotCommands(commands, true, this->mode);
}

void RobotHub::processRobotCommands(const proto::RobotCommands &commands, bool forTeamYellow, RobotHubMode mode) {
    switch (mode) {
        case RobotHubMode::SIMULATOR:
            this->sendCommandsToSimulator(commands, forTeamYellow);
            break;
        case RobotHubMode::BASESTATION:
            this->sendCommandsToBasestation(commands, forTeamYellow);
            break;
        case RobotHubMode::BOTH:
            this->sendCommandsToSimulator(commands, forTeamYellow);
            this->sendCommandsToBasestation(commands, forTeamYellow);
            break;
        default:
            std::cout << "WARNING: Unknown robothub mode" << std::endl;
            break;
    }
}

void RobotHub::onSettingsFromChannel1(const proto::Settings &settings) { this->settings = settings; }

/* Unsafe function that can cause data races in commands_sent and feedback_received,
    as it is updated from multiple threads without guards. This should not matter
    however, as these variables are just for debugging purposes. */
void RobotHub::printStatistics() {
    std::stringstream ss;

    const int amountOfColumns = 4;
    for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i += amountOfColumns) {
        for (int j = 0; j < amountOfColumns; j++) {
            const int robotId = i + j;
            if (robotId < MAX_AMOUNT_OF_ROBOTS) {
                int nSent = commands_sent[robotId];
                int nReceived = feedback_received[robotId];
                commands_sent[robotId] = 0;
                feedback_received[robotId] = 0;

                if (robotId < 10) ss << " ";
                ss << robotId;
                ss << "(";
                if (nSent < 100) ss << " ";
                if (nSent < 10) ss << " ";
                ss << nSent;
                ss << ":";
                if (nReceived < 100) ss << " ";
                if (nReceived < 10) ss << " ";
                ss << nReceived;
                ss << ") | ";
            }
        }
        ss << std::endl;
    }
    std::cout << ss.str();
}

void RobotHub::handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback) {
    proto::RobotFeedback feedbackToBePublished;
    feedbackToBePublished.set_is_from_team_yellow(feedback.isTeamYellow);

    proto::SimulatorRobotFeedback* feedbackOfRobots = feedbackToBePublished.mutable_simulator_robot_feedback();

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        proto::SimulatorRobotFeedback_Robot *feedbackOfRobot = feedbackOfRobots->add_robots();
        feedbackOfRobot->set_id(robotId);
        feedbackOfRobot->set_has_ball(hasBall);
    }

    this->robotFeedbackPublisher->publish(feedbackToBePublished);
}

void RobotHub::handleRobotFeedbackFromBasestation(const RobotFeedback &feedback) {
    proto::RobotFeedback feedbackToBePublished;
    //TODO: Get from basestation which color

    proto::BasestationRobotFeedback *feedbackOfRobot = feedbackToBePublished.mutable_basestation_robot_feedback();
    feedbackOfRobot->set_id(feedback.id);
    feedbackOfRobot->set_xsens_is_calibrated(feedback.XsensCalibrated);
    feedbackOfRobot->set_ball_sensor_is_working(feedback.ballSensorWorking);
    feedbackOfRobot->set_has_ball(feedback.hasBall);
    feedbackOfRobot->set_ball_position(feedback.ballPos);
    feedbackOfRobot->set_capacitor_is_charged(feedback.capacitorCharged);
    feedbackOfRobot->set_speed(feedback.rho);
    feedbackOfRobot->set_direction(feedback.theta);
    feedbackOfRobot->set_angle(feedback.angle);
    feedbackOfRobot->set_battery_level(feedback.batteryLevel);
    feedbackOfRobot->set_signal_strength(feedback.rssi);
    feedbackOfRobot->set_has_locked_wheel(feedback.wheelLocked);

    this->robotFeedbackPublisher->publish(feedbackToBePublished);
}

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.printStatistics();
    }
}