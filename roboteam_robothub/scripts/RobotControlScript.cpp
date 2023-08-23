#include <RobotCommandsNetworker.hpp>
#include <SettingsNetworker.hpp>
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "RobotHubMode.h"

using namespace rtt::net;

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr float MAX_ANGULAR_VELOCITY = 2.0f;
constexpr float MAX_DRIBBLER_SPEED = 100;
constexpr float MAX_VELOCITY = 1.0f;
constexpr int COMMAND_TRANSMISSION_INTERVAL_MS = 100;
constexpr int SETTINGS_TRANSMISSION_INTERVAL_MS = 1000;

///////////////////////////////////////////////////////////
/// All things related to the sending of robot commands ///
///////////////////////////////////////////////////////////
enum class Rotation {
    NO_ROTATION,
    LEFT_ROTATION,
    RIGHT_ROTATION
};
enum class Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    SINE_X,
    SINE_Y,
    SINE_CIRCLE,
    STILL,
};
enum class Team {
    NEITHER,
    YELLOW,
    BLUE,
    BOTH
};

bool shouldSendRobotCommands = false;
bool shouldAskCLICommands = false;
bool shouldSendSettings = false;

auto lastTimeCommandWasUsed = std::chrono::steady_clock::now();

rtt::net::RobotHubMode currentMode = rtt::net::RobotHubMode::SIMULATOR;
Team currentTeam = Team::NEITHER;
Rotation currentRotation = Rotation::NO_ROTATION;
Movement currentMovement = Movement::STILL;

int targetRobotID = 0;
bool targetAllRobotIDs = false;
rtt::Vector2 targetVelocity(0, 0);
rtt::Angle targetAngle(0.0);
double targetAngularVelocity = 0.0;
bool useDribbler = false;
bool useAngularVelocity = false;

void updateCommandValues() {
    auto now = std::chrono::steady_clock::now();
    auto timeSinceLastCommandMS = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastTimeCommandWasUsed).count();
    auto timeSinceLastCommand = static_cast<double>(timeSinceLastCommandMS) / 1000;

    if (currentRotation == Rotation::LEFT_ROTATION) {
        targetAngle = rtt::Angle(timeSinceLastCommand * MAX_ANGULAR_VELOCITY);
        targetAngularVelocity = MAX_ANGULAR_VELOCITY;
    } else if (currentRotation == Rotation::RIGHT_ROTATION) {
        targetAngle = rtt::Angle(timeSinceLastCommand * MAX_ANGULAR_VELOCITY * -1);
        targetAngularVelocity = -MAX_ANGULAR_VELOCITY;
    } else {
        targetAngle = rtt::Angle(0.0);
        targetAngularVelocity = 0.0;
    }

    if (currentMovement == Movement::SINE_X)
        targetVelocity = rtt::Vector2(std::sin(timeSinceLastCommand), 0.0);
    else if (currentMovement == Movement::SINE_Y)
        targetVelocity = rtt::Vector2(0.0, std::sin(timeSinceLastCommand));
    else if (currentMovement == Movement::SINE_CIRCLE)
        targetVelocity = rtt::Vector2(std::cos(timeSinceLastCommand), std::sin(timeSinceLastCommand)).stretchToLength(MAX_VELOCITY);
    else if (currentMovement == Movement::FORWARD)
        targetVelocity = rtt::Vector2(MAX_VELOCITY, 0.0);
    else if (currentMovement == Movement::BACKWARD)
        targetVelocity = rtt::Vector2(-MAX_VELOCITY, 0.0);
    else if (currentMovement == Movement::LEFT)
        targetVelocity = rtt::Vector2(0.0, MAX_VELOCITY);
    else if (currentMovement == Movement::RIGHT)
        targetVelocity = rtt::Vector2(0.0, -MAX_VELOCITY);
    else
        targetVelocity = rtt::Vector2(0.0, 0.0);
}

rtt::RobotCommand createCommandForRobot(int id) {
    rtt::RobotCommand cmd = {
        .id = id,
        .velocity = targetVelocity,
        .targetAngle = targetAngle,
        .targetAngularVelocity = targetAngularVelocity,
        .useAngularVelocity = useAngularVelocity,

        .cameraAngleOfRobot = 0.0,
        .cameraAngleOfRobotIsSet = false,

        .kickSpeed = 0.0,
        .waitForBall = false,
        .kickType = rtt::KickType::NO_KICK,

        .dribblerSpeed = useDribbler ? MAX_DRIBBLER_SPEED : 0.0f,

        .ignorePacket = false
    };
    return cmd;
}

rtt::RobotCommands getRobotCommandsForTeam() {
    updateCommandValues();

    rtt::RobotCommands commands;
    if (targetAllRobotIDs) {
        for (int i = 0; i < MAX_AMOUNT_OF_ROBOTS; i++) {
            commands.push_back(createCommandForRobot(i));
        }
    } else {
        commands.push_back(createCommandForRobot(targetRobotID));
    }
    return commands;
}

void runCommandSending() {
    auto commandsPublisherBlue = RobotCommandsBluePublisher();
    auto commandsPublisherYellow = RobotCommandsYellowPublisher();

    shouldSendRobotCommands = true;

    while (shouldSendRobotCommands) {
        auto commands = getRobotCommandsForTeam();

        if (currentTeam == Team::BLUE) {
            commandsPublisherBlue.publish(commands);
        } else if (currentTeam == Team::YELLOW) {
            commandsPublisherYellow.publish(commands);
        } else if (currentTeam == Team::BOTH) {
            commandsPublisherYellow.publish(commands);
            commandsPublisherBlue.publish(commands);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(COMMAND_TRANSMISSION_INTERVAL_MS));
    }
}

void runSettingsSending() {
    auto settingsPublisher = SettingsPublisher();

    shouldSendSettings = true;

    while (shouldSendSettings) {
        proto::GameSettings settings;
        settings.set_robot_hub_mode(rtt::net::robotHubModeToProto(currentMode));

        settingsPublisher.publish(settings);

        std::this_thread::sleep_for(std::chrono::milliseconds(SETTINGS_TRANSMISSION_INTERVAL_MS));
    }
}

/////////////////////////////////////
/// All things related to the CLI ///
/////////////////////////////////////

void printCommandOptions() {
    std::cout << "The commands you can enter are:" << std::endl
              << "- <nothing>   : Halts all robots. Spam enter in case of emergency" << std::endl
              << "- basestation : Changes RobotHub to basestation mode" << std::endl
              << "- simulator   : Changes RobotHub to simulation mode" << std::endl
              << "- neither     : Sends commands to neither team" << std::endl
              << "- yellow      : Sends commands to yellow team" << std::endl
              << "- blue        : Sends commands to blue team" << std::endl
              << "- both        : Sends commands to both teams" << std::endl
              << "- 0..15       : Sends command only to typed id" << std::endl
              << "- all         : Sends same command to all robot ids" << std::endl
              << "- rotLeft     : Makes robots rotate left" << std::endl
              << "- rotLeft     : Makes robots rotate left" << std::endl
              << "- rotRight    : Makes robots rotate right" << std::endl
              << "- rotStop     : Makes robot stop rotation" << std::endl
              << "- angVel      : Toggles sending rotation in angular velocity" << std::endl
              << "- forward     : Moves robot forward" << std::endl
              << "- backward    : Moves robot backward" << std::endl
              << "- left        : Moves robot left" << std::endl
              << "- right       : Moves robot right" << std::endl
              << "- sinex       : Moves robot back and forth on x axis" << std::endl
              << "- siney       : Moves robot back and forth on x axis" << std::endl
              << "- circle      : Moves robot back in a circle" << std::endl
              << "- still       : Set robot speed to 0" << std::endl
              << "- dribble     : Toggles the robots dribbler" << std::endl
              << "- halt        : Completely freezes the robot" << std::endl
              << "- stop        : Stops this script" << std::endl
              << "- help        : For displaying this message" << std::endl;
}

int stringToRobotId(std::string s) {
    if (s == "0") {
        return 0;
    } else if (s == "1") {
        return 1;
    } else if (s == "2") {
        return 2;
    } else if (s == "3") {
        return 3;
    } else if (s == "4") {
        return 4;
    } else if (s == "5") {
        return 5;
    } else if (s == "6") {
        return 6;
    } else if (s == "7") {
        return 7;
    } else if (s == "8") {
        return 8;
    } else if (s == "9") {
        return 9;
    } else if (s == "10") {
        return 10;
    } else if (s == "11") {
        return 11;
    } else if (s == "12") {
        return 12;
    } else if (s == "13") {
        return 13;
    } else if (s == "14") {
        return 14;
    } else if (s == "15") {
        return 15;
    } else {
        return -1;
    }
}

bool handleCommand(std::string cmd) {
    // Convert command to upper case
    for (auto& c : cmd)
        c = static_cast<char>(toupper(c));

    // Now handle command
    if (cmd.empty()) {  // Abort, freeze, stop all robots aaaaah
        targetAllRobotIDs = true;
        currentMovement = Movement::STILL;
        currentRotation = Rotation::NO_ROTATION;
        currentTeam = Team::BOTH;
        useDribbler = false;
    } else if (cmd == "BASESTATION") {
        currentMode = rtt::net::RobotHubMode::BASESTATION;
    } else if (cmd == "SIMULATION") {
        currentMode = rtt::net::RobotHubMode::SIMULATOR;
    } else if (cmd == "NEITHER") {
        currentTeam = Team::NEITHER;
    } else if (cmd == "YELLOW") {
        currentTeam = Team::YELLOW;
    } else if (cmd == "BLUE") {
        currentTeam = Team::BLUE;
    } else if (cmd == "BOTH") {
        currentTeam = Team::BOTH;
    } else if (stringToRobotId(cmd) >= 0) {
        targetRobotID = stringToRobotId(cmd);
        targetAllRobotIDs = false;
    } else if (cmd == "ALL") {
        targetAllRobotIDs = true;
    } else if (cmd == "ROTLEFT") {
        currentRotation = Rotation::LEFT_ROTATION;
        targetAngularVelocity = MAX_ANGULAR_VELOCITY;
    } else if (cmd == "ROTRIGHT") {
        currentRotation = Rotation::RIGHT_ROTATION;
        targetAngularVelocity = -MAX_ANGULAR_VELOCITY;
    } else if (cmd == "ROTSTOP") {
        currentRotation = Rotation::NO_ROTATION;
    } else if (cmd == "ANGVEL") {
        useAngularVelocity = !useAngularVelocity;
    } else if (cmd == "FORWARD") {
        currentMovement = Movement::FORWARD;
    } else if (cmd == "BACKWARD") {
        currentMovement = Movement::BACKWARD;
    } else if (cmd == "LEFT") {
        currentMovement = Movement::LEFT;
    } else if (cmd == "RIGHT") {
        currentMovement = Movement::RIGHT;
    } else if (cmd == "SINEX") {
        currentMovement = Movement::SINE_X;
    } else if (cmd == "SINEY") {
        currentMovement = Movement::SINE_Y;
    } else if (cmd == "CIRCLE") {
        currentMovement = Movement::SINE_CIRCLE;
    } else if (cmd == "STILL") {
        currentMovement = Movement::STILL;
    } else if (cmd == "DRIBBLE") {
        useDribbler = !useDribbler;
        return false;
    } else if (cmd == "STOP") {
        shouldSendSettings = false;
        shouldSendRobotCommands = false;
        shouldAskCLICommands = false;
    } else if (cmd == "HELP") {
        printCommandOptions();
        return false;
    } else {
        std::cout << "Unknown command. Type 'help' for more information." << std::endl;
        return false;
    }
    return true;
}

std::string currentMovementToString() {
    switch (currentMovement) {
        case Movement::STILL:
            return "still";
        case Movement::FORWARD:
            return "forward";
        case Movement::BACKWARD:
            return "backward";
        case Movement::LEFT:
            return "left";
        case Movement::RIGHT:
            return "right";
        case Movement::SINE_X:
            return "sine-x";
        case Movement::SINE_Y:
            return "sine-y";
        case Movement::SINE_CIRCLE:
            return "circle";
        default:
            return "UNKNOWN";
    }
}
std::string currentRotationToString() {
    std::string method = useAngularVelocity ? "in angular velocity" : "in absolute angles";

    switch (currentRotation) {
        case Rotation::NO_ROTATION:
            return "without rotating";
        case Rotation::LEFT_ROTATION:
            return "while turning left " + method;
        case Rotation::RIGHT_ROTATION:
            return "while turning right " + method;
        default:
            return "UNKNOWN";
    }
}

std::string targetIDsToString() {
    if (targetAllRobotIDs)
        return "all robots";
    else
        return "id " + std::to_string(targetRobotID);
}

std::string targetTeamToString() {
    switch (currentTeam) {
        case Team::NEITHER:
            return "neither team";
        case Team::YELLOW:
            return "yellow team";
        case Team::BLUE:
            return "blue team";
        case Team::BOTH:
            return "both teams";
        default:
            return "UNKNOWN team";
    }
}

void printStatus() {
    std::cout << "Sending "
              << currentMovementToString()
              << " command "
              << currentRotationToString()
              << " to "
              << targetIDsToString()
              << " of "
              << targetTeamToString()
              << " from "
              << rtt::net::robotHubModeToString(currentMode)
              << std::endl;
}

void runCommandLineInterface() {
    std::cout << "This script can be used to manually control robots. Make sure RobotHub is running" << std::endl;
    printCommandOptions();

    shouldAskCLICommands = true;

    while (shouldAskCLICommands) {
        printStatus();
        std::cout << "Enter command: ";
        std::string enteredCommand;
        std::getline(std::cin, enteredCommand);
        if (handleCommand(enteredCommand)) {
            lastTimeCommandWasUsed = std::chrono::steady_clock::now();
        }
    }
    std::cout << "I hope this script helped you. Bye!" << std::endl;
}

/////////////////////////
/// THE MAIN FUNCTION ///
/////////////////////////

int main() {
    std::thread settingsTransmitter(runSettingsSending);
    std::thread commandsTransmitter(runCommandSending);
    std::thread interfaceThread(runCommandLineInterface);

    if (interfaceThread.joinable()) {
        interfaceThread.join();
    }
    if (settingsTransmitter.joinable()) {
        settingsTransmitter.join();
    }
    if (commandsTransmitter.joinable()) {
        commandsTransmitter.join();
    }

    return 0;
}