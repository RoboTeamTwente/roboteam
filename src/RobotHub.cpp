#include "RobotHub.h"

#include <roboteam_proto/State.pb.h>
#include <basestation/Packing.h>
#include <iostream>
#include <sstream>

namespace rtt {
namespace robothub {

RobotHub::RobotHub() {
    std::cout << "[RobotHub] New RobotHub" << std::endl;

    simulation::SimulatorNetworkConfiguration config = {
        .blueFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_BLUE_CONTROL,
        .yellowFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_YELLOW_CONTROL,
        .configurationFeedbackPort = DEFAULT_GRSIM_FEEDBACK_PORT_CONFIGURATION
    };
    
    this->simulatorManager = std::make_unique<simulation::SimulatorManager>(config);
    this->simulatorManager->setRobotControlFeedbackCallback(handleRobotFeedbackFromSimulator);
    
    this->basestationManager = std::make_unique<basestation::BasestationManager>();
    this->basestationManager->setFeedbackCallback(handleRobotFeedbackFromBasestation);
}

int RobotHub::run() {
    while (true) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        printStatistics();
    }
    return 0;
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

void RobotHub::sendCommandsToSimulator(proto::AICommand &aiCmd) {
    if (this->simulatorManager == nullptr) return;

    simulation::RobotControlCommand simCommand;
    for (auto robotCommand : aiCmd.commands()) {
        int id = robotCommand.id();
        float kickSpeed = robotCommand.kicker() ? DEFAULT_KICK_SPEED : 0.0f;
        float kickAngle = 0.0f;
        float dribblerSpeed = (float)robotCommand.dribbler();
        float xVelocity = robotCommand.vel().x();
        float yVelocity = robotCommand.vel().y();
        float angularVelocity = robotCommand.w();

        simCommand.addRobotControlWithGlobalSpeeds(id, kickSpeed, kickAngle, dribblerSpeed, xVelocity, yVelocity, angularVelocity);

        // Update statistics
        this->commands_sent[id]++;
    }

    bool isCommandForTeamYellow = this->settings.isyellow();
    this->simulatorManager->sendRobotControlCommand(simCommand, isCommandForTeamYellow);
}

void RobotHub::sendCommandsToBasestation(proto::AICommand &aiCmd) {
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
        this->sendCommandsToSimulator(AIcmd);
    }
}

void RobotHub::processSettings(proto::Setting &_settings) { settings = _settings; }

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

void handleRobotFeedbackFromSimulator(simulation::RobotControlFeedback& feedback) {
    std::cout << "Received robot feedback from the simulator!" << std::endl;
}
void handleRobotFeedbackFromBasestation(RobotFeedback& feedback) {
    std::cout << "Received robot feedback from the basestation!" << std::endl;
}

}  // namespace robothub
}  // namespace rtt

int main(int argc, char *argv[]) {
    rtt::robothub::RobotHub app;
    return app.run();
}