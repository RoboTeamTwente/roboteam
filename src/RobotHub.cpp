#include <RobotCommand.h>
#include <RobotHub.h>

#include <cmath>
#include <iostream>
#include <sstream>

namespace rtt::robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    simulation::SimulatorNetworkConfiguration config = {.blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
                                                        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
                                                        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION};

    if (!this->subscribe()) {
        throw FailedToInitializeNetworkersException();
    }

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    auto simulationFeedbackCallback = std::bind(&RobotHub::handleRobotFeedbackFromSimulator, this, std::placeholders::_1);
    this->simulatorManager->setRobotControlFeedbackCallback(simulationFeedbackCallback);

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    auto basestationFeedbackCallback = std::bind(&RobotHub::handleRobotFeedbackFromBasestation, this, std::placeholders::_1);
    this->basestationManager->setFeedbackCallback(basestationFeedbackCallback);
}

bool RobotHub::subscribe() {
    auto blueCommandsCallback = std::bind(&RobotHub::onBlueRobotCommands, this, std::placeholders::_1);
    this->robotCommandsBlueSubscriber = std::make_unique<rtt::net::RobotCommandsBlueSubscriber>(blueCommandsCallback);

    auto yellowCommandsCallback = std::bind(&RobotHub::onYellowRobotCommands, this, std::placeholders::_1);
    this->robotCommandsYellowSubscriber = std::make_unique<rtt::net::RobotCommandsYellowSubscriber>(yellowCommandsCallback);

    auto settingsCallback = std::bind(&RobotHub::onSettings, this, std::placeholders::_1);
    this->settingsSubscriber = std::make_unique<rtt::net::SettingsSubscriber>(settingsCallback);

    this->robotFeedbackPublisher = std::make_unique<rtt::net::RobotFeedbackPublisher>();

    // All networkers should not be a nullptr
    return this->robotCommandsBlueSubscriber != nullptr && this->robotCommandsYellowSubscriber != nullptr && this->settingsSubscriber != nullptr &&
           this->robotFeedbackPublisher != nullptr;
}

void RobotHub::sendCommandsToSimulator(const proto::AICommand &commands, bool toTeamYellow) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : commands.commands()) {
        int id = robotCommand.id();
        float kickSpeed = robotCommand.chip_kick_vel();
        float kickAngle = robotCommand.chipper() ? DEFAULT_CHIPPER_ANGLE : 0.0f;
        float dribblerSpeed = (robotCommand.dribbler() > 0 ? MAX_DRIBBLER_SPEED : 0.0);  // dribbler_speed is range of 0 to 1
        float xVelocity = robotCommand.vel().x();
        float yVelocity = robotCommand.vel().y();
        // TODO: Check if there is angular velocity
        float angularVelocity = robotCommand.w();

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update statistics
        this->commands_sent[id]++;
    }

    this->simulatorManager->sendRobotControlCommand(simCommand, toTeamYellow);
}

void RobotHub::sendCommandsToBasestation(const proto::AICommand &commands, bool toTeamYellow) {
    for (const proto::RobotCommand &protoCommand : commands.commands()) {
        // Convert the proto::RobotCommand to a RobotCommand for the basestation

        float rho = sqrtf(protoCommand.vel().x() * protoCommand.vel().x() + protoCommand.vel().y() * protoCommand.vel().y());
        float theta = atan2f(protoCommand.vel().y(), protoCommand.vel().x());
        auto bot = rtt::robothub::utils::getWorldBot(protoCommand.id(), toTeamYellow, world);

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
            command.cameraAngle = bot->w();
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

void RobotHub::onBlueRobotCommands(const proto::AICommand &commands) { this->processRobotCommands(commands, false, this->mode); }
void RobotHub::onYellowRobotCommands(const proto::AICommand &commands) { this->processRobotCommands(commands, true, this->mode); }

void RobotHub::processRobotCommands(const proto::AICommand &commands, bool forTeamYellow, RobotHubMode mode) {
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

void RobotHub::onSettings(const proto::Setting &settings) {
    this->settings = settings;
    this->mode = settings.serialmode() ? RobotHubMode::BASESTATION : RobotHubMode::SIMULATOR;
}

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

    // proto::RobotFeedback* feedbackOfRobots = feedbackToBePublished.mutable_receivedfeedback();

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
        feedbackOfRobot->set_id(robotId);
        feedbackOfRobot->set_hasball(hasBall);
    }

    this->sendRobotFeedback(feedbackToBePublished);
}

void RobotHub::handleRobotFeedbackFromBasestation(const RobotFeedback &feedback) {
    proto::RobotData feedbackToBePublished;
    // TODO: Get from basestation which color

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
}

bool RobotHub::sendRobotFeedback(const proto::RobotData &feedback) { return this->robotFeedbackPublisher->publish(feedback); }

const char *FailedToInitializeNetworkersException::what() const throw() { return "Failed to initialize networker(s). Is another RobotHub running?"; }

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.printStatistics();
    }
}