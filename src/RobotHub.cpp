#include <RobotHub.h>
#include <basestation/Packing.h>

#include <cmath>
#include <iostream>
#include <sstream>

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
    // TODO: choose either _PRIMARY_CHANNEL or _SECONDARY_CHANNEL based on some flag somewhere
    robotCommandSubscriber = std::make_unique<proto::Subscriber<proto::AICommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL, &RobotHub::onRobotCommandsFromChannel1, this);
    robotCommandSubscriber2 = std::make_unique<proto::Subscriber<proto::AICommand>>(proto::ROBOT_COMMANDS_SECONDARY_CHANNEL, &RobotHub::onRobotCommandsFromChannel2, this);

    settingsSubscriber = std::make_unique<proto::Subscriber<proto::Setting>>(proto::SETTINGS_PRIMARY_CHANNEL, &RobotHub::onSettingsFromChannel1, this);
    settingsSubscriber2 = std::make_unique<proto::Subscriber<proto::Setting>>(proto::SETTINGS_SECONDARY_CHANNEL, &RobotHub::onSettingsFromChannel2, this);

    feedbackPublisher = std::make_unique<proto::Publisher<proto::RobotData>>(proto::FEEDBACK_PRIMARY_CHANNEL);
}

void RobotHub::sendCommandsToSimulator(const proto::AICommand &commands, bool toTeamYellow) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : commands.commands()) {
        int id = robotCommand.id();
        float kickSpeed = robotCommand.chipper() || robotCommand.kicker() ? robotCommand.chip_kick_vel() : 0.0f;
        float kickAngle = robotCommand.chipper() ? DEFAULT_CHIPPER_ANGLE : 0.0f;
        float dribblerSpeed = robotCommand.dribbler() > 0 ? MAX_DRIBBLER_SPEED : 0.0f;
        float xVelocity = robotCommand.vel().x();
        float yVelocity = robotCommand.vel().y();
        float angularVelocity = robotCommand.w();

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update statistics
        this->commands_sent[id]++;
    }

    this->simulatorManager->sendRobotControlCommand(simCommand, toTeamYellow);
}

void RobotHub::sendCommandsToBasestation(const proto::AICommand &commands, bool toTeamYellow) {
    for (const proto::RobotCommand &command : commands.commands()) {
        // Convert the proto::RobotCommand to a RobotCommandPayload
        RobotCommandPayload payload = createEmbeddedCommand(command, commands.extrapolatedworld(), toTeamYellow);

        this->basestationManager->sendSerialCommand(payload);

        // Update statistics
        commands_sent[command.id()]++;
    }
}

void RobotHub::onRobotCommandsFromChannel1(proto::AICommand& commands) {
    this->processRobotCommands(commands, this->settingsFromChannel1.isyellow(), this->settingsFromChannel1.serialmode());
}
void RobotHub::onRobotCommandsFromChannel2(proto::AICommand& commands) {
    this->processRobotCommands(commands, this->settingsFromChannel2.isyellow(), this->settingsFromChannel2.serialmode());
}

void RobotHub::processRobotCommands(proto::AICommand &commands, bool forTeamYellow, bool useSimulator) {
    if (useSimulator) {
        this->sendCommandsToSimulator(commands, forTeamYellow);
    } else {
        this->sendCommandsToBasestation(commands, forTeamYellow);
    }
}

void RobotHub::onSettingsFromChannel1(proto::Setting &settings) { this->settingsFromChannel1 = settings; }
void RobotHub::onSettingsFromChannel2(proto::Setting &settings) { this->settingsFromChannel2 = settings; }

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
    proto::RobotData feedbackToBePublished;
    feedbackToBePublished.set_isyellow(feedback.isTeamYellow);

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
        feedbackOfRobot->set_id(robotId);
        feedbackOfRobot->set_hasball(hasBall);
    }

    this->feedbackPublisher->send(feedbackToBePublished);
}

void RobotHub::handleRobotFeedbackFromBasestation(const RobotFeedback &feedback) {
    proto::RobotData feedbackToBePublished;
    //feedbackToBePublished.set_isyellow(this->settings.isyellow()); //TODO: Get from basestation which team it owns

    proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
    feedbackOfRobot->set_id(feedback.id);
    feedbackOfRobot->set_xsenscalibrated(feedback.XsensCalibrated);
    feedbackOfRobot->set_ballsensorisworking(feedback.ballSensorWorking);
    feedbackOfRobot->set_batterylow(feedback.batteryLevel <= BATTERY_LOW_LEVEL);
    feedbackOfRobot->set_hasball(feedback.hasBall);
    feedbackOfRobot->set_ballpos(feedback.ballPos);
    // Convert polar velocity to cartesian velocity
    feedbackOfRobot->set_x_vel(feedback.rho * std::cos(feedback.theta));
    feedbackOfRobot->set_yaw(feedback.angle);
    // Convert polar velocity to cartesian velocity
    feedbackOfRobot->set_y_vel(feedback.rho * std::sin(feedback.theta));
    feedbackOfRobot->set_haslockedwheel(feedback.wheelLocked > 0);
    feedbackOfRobot->set_signalstrength((float)feedback.rssi);

    this->feedbackPublisher->send(feedbackToBePublished);
}

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.printStatistics();
    }
}