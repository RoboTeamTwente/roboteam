#include "RobotHub.h"

#include <basestation/Packing.h>
#include <roboteam_proto/State.pb.h>

#include <iostream>
#include <sstream>

namespace rtt::robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    simulation::SimulatorNetworkConfiguration config = {.blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
                                                        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
                                                        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION};

    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    this->simulatorManager->setRobotControlFeedbackCallback(handleRobotFeedbackFromSimulator);

    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    this->basestationManager->setFeedbackCallback(handleRobotFeedbackFromBasestation);

    this->subscribe();
}

void RobotHub::subscribe() {
    // TODO: choose either _PRIMARY_CHANNEL or _SECONDARY_CHANNEL based on some flag somewhere

    robotCommandSubscriber = std::make_unique<proto::Subscriber<proto::AICommand>>(proto::ROBOT_COMMANDS_PRIMARY_CHANNEL, &RobotHub::processAIcommand, this);

    //    worldStateSubscriber = std::make_unique<proto::Subscriber<proto::State>>(
    //            proto::WORLD_CHANNEL, &RobotHub::processWorldState, this
    //    );

    settingsSubscriber = std::make_unique<proto::Subscriber<proto::Setting>>(proto::SETTINGS_PRIMARY_CHANNEL, &RobotHub::processSettings, this);

    feedbackPublisher = std::make_unique<proto::Publisher<proto::RobotFeedback>>(proto::FEEDBACK_PRIMARY_CHANNEL);
}

void RobotHub::sendCommandsToSimulator(const proto::AICommand &aiCmd, const proto::World &world) {
    if (this->simulatorManager == nullptr) return;

    bool isCommandForTeamYellow = this->settings.isyellow();

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : aiCmd.commands()) {
        int id = robotCommand.id();
        float kick_vel = robotCommand.chipper() || robotCommand.kicker() ? robotCommand.chip_kick_vel() : 0.0f;
        float kick_angle = robotCommand.chipper() ? 45.0f : 0.0f;
        float dribbler_speed = robotCommand.dribbler() > 0 ? 1021.0f : 0.0f;

        float forwardVelocity = 0.0f;
        float leftVelocity = 0.0f;
        float angularVelocity = 0.0f;

        {
            float robot_angle = 0.0f;
            std::shared_ptr<proto::WorldRobot> findBot = rtt::robothub::utils::getWorldBot(robotCommand.id(), isCommandForTeamYellow, world);
            if (findBot) {
                robot_angle = -findBot->angle();
            } else {
                std::cout << "could not find robot!" << std::endl;
            }
            float forward = cosf(robot_angle) * robotCommand.vel().x() - sinf(robot_angle) * robotCommand.vel().y();
            float left = sinf(robot_angle) * robotCommand.vel().x() + cosf(robot_angle) * robotCommand.vel().y();

            forwardVelocity = forward;
            leftVelocity = left;
            angularVelocity = robotCommand.w();
        }

        simCommand.addRobotControlWithLocalSpeeds(id, kick_vel, kick_angle, dribbler_speed, forwardVelocity, leftVelocity, angularVelocity);

        // simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update statistics
        this->commands_sent[id]++;
    }

    this->simulatorManager->sendRobotControlCommand(simCommand, isCommandForTeamYellow);
}

void RobotHub::sendCommandsToBasestation(const proto::AICommand &aiCmd) {
    for (const proto::RobotCommand &cmd : aiCmd.commands()) {
        // Convert the proto::RobotCommand to a RobotCommandPayload
        RobotCommandPayload payload = createEmbeddedCommand(cmd, aiCmd.extrapolatedworld(), settings.isyellow());

        this->basestationManager->sendSerialCommand(payload);

        // Update statistics
        commands_sent[cmd.id()]++;
    }
}

void RobotHub::processAIcommand(proto::AICommand &AIcmd) {
    if (settings.serialmode()) {
        this->sendCommandsToBasestation(AIcmd);
    } else {
        this->sendCommandsToSimulator(AIcmd, AIcmd.extrapolatedworld());
    }
}

void RobotHub::processSettings(proto::Setting &_settings) { settings = _settings; }

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

void handleRobotFeedbackFromSimulator(const simulation::RobotControlFeedback &feedback) {
    // std::cout << "Received robot feedback from the simulator!" << std::endl;
    // TODO: Forward feedback to AI or something idk
}
void handleRobotFeedbackFromBasestation(const RobotFeedback &feedback) {
    // std::cout << "Received robot feedback from the basestation!" << std::endl;
    // TODO: Forward feedback to AI or something idk
}

}  // namespace rtt::robothub

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;

    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        app.printStatistics();
    }
}