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

    if (!this->subscribe()) {
        throw FailedToInitializeNetworkersException();
    }

    this->mode = utils::RobotHubMode::NEITHER;
    std::cout << "[RobotHub]: Starting with default mode of: " << utils::modeToString(this->mode) << std::endl;

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    this->simulatorManager->setRobotControlFeedbackCallback([&](const simulation::RobotControlFeedback &feedback) { this->handleRobotFeedbackFromSimulator(feedback); });

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    this->basestationManager->setFeedbackCallback([&](const RobotFeedback &feedback, utils::TeamColor color) { this->handleRobotFeedbackFromBasestation(feedback, color); });
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

void RobotHub::sendCommandsToSimulator(const proto::AICommand &commands, utils::TeamColor color) {
    if (this->simulatorManager == nullptr) return;

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

        // Update statistics
        this->commands_sent[id]++;
    }

    this->simulatorManager->sendRobotControlCommand(simCommand, color);
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
            command.cameraAngle = bot->w();
        } else {
            command.useCameraAngle = false;
            command.cameraAngle = 0.0;
        }

        command.feedback = false;

        this->basestationManager->sendRobotCommand(command, color);

        // Update statistics
        commands_sent[protoCommand.id()]++;
    }
}

void RobotHub::onBlueRobotCommands(const proto::AICommand &commands) { this->processRobotCommands(commands, utils::TeamColor::BLUE, this->mode); }
void RobotHub::onYellowRobotCommands(const proto::AICommand &commands) { this->processRobotCommands(commands, utils::TeamColor::YELLOW, this->mode); }

void RobotHub::processRobotCommands(const proto::AICommand &commands, utils::TeamColor color, utils::RobotHubMode mode) {
    switch (mode) {
        case utils::RobotHubMode::SIMULATOR:
            this->sendCommandsToSimulator(commands, color);
            break;
        case utils::RobotHubMode::BASESTATION:
            this->sendCommandsToBasestation(commands, color);
            break;
        case utils::RobotHubMode::BOTH:
            this->sendCommandsToSimulator(commands, color);
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
    if (newMode != this->mode) {
        std::cout << "[RobotHub]: Changed robothub mode from: " << utils::modeToString(this->mode) << " to: " << utils::modeToString(newMode) << std::endl;
    }

    this->mode = newMode;
}

/* Unsafe function that can cause data races in commands_sent and feedback_received,
    as it is updated from multiple threads without guards. This should not matter
    however, as these variables are just for debugging purposes. */
void RobotHub::printStatistics() {
    if (this->basestationManager == nullptr) {
        std::cout << "Basestation manager is not initialized" << std::endl;
        return;
    }

    this->basestationManager->printStatus();

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
    feedbackToBePublished.set_isyellow(feedback.color == utils::TeamColor::YELLOW);

    // proto::RobotFeedback* feedbackOfRobots = feedbackToBePublished.mutable_receivedfeedback();

    for (auto const &[robotId, hasBall] : feedback.robotIdHasBall) {
        proto::RobotFeedback *feedbackOfRobot = feedbackToBePublished.add_receivedfeedback();
        feedbackOfRobot->set_id(robotId);
        feedbackOfRobot->set_hasball(hasBall);
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
        app.printStatistics();
    }
}