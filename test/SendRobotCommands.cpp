#include <RobotCommandsNetworker.hpp>
#include <RobotFeedbackNetworker.hpp>
#include <SettingsNetworker.hpp>

#include <iostream>
#include <memory>
#include <chrono>
#include <thread>
#include <string>

using namespace rtt::net;

constexpr int MAX_AMOUNT_OF_ROBOTS = 16;
constexpr float ROTATE_SPEED = 2.0f;

enum Command {
    INVALID,
    HELP,

    MODE_TO_BASESTATION,
    MODE_TO_SIMULATOR,

    SEND_ROTATE_LEFT_COMMANDS,
    SEND_ROTATE_RIGHT_COMMANDS,
    SEND_HALT_COMMANDS,

    DO_NOT_SEND,
    SEND_TO_YELLOW,
    SEND_TO_BLUE,
    SEND_TO_BOTH,

    ROBOT_ID,
    ALL_ROBOTS,

    STOP
};

enum Mode {
    BASESTATION, SIMULATOR
};
enum RobotCommand {
    HALT, LEFT, RIGHT
};
enum Team {
    NEITHER, YELLOW, BLUE, BOTH
};

Mode mode = Mode::BASESTATION;
RobotCommand robotCommand = RobotCommand::HALT;
Team team = Team::NEITHER;
int robotId = 0;
bool sendToAllRobots = false;

bool shouldRunCommands = true;
bool shouldAskCommands = true;

void printCommandOptions() {
    std::cout << "The commands you can enter are:" << std::endl
              << "- basestation : For changing RobotHub to basestation mode" << std::endl
              << "- simulator   : For changing RobotHub to simulation mode" << std::endl
              << "- left        : For sending robot commands to rotate to the left" << std::endl
              << "- right       : For sending robot commands to rotate to the right" << std::endl
              << "- halt        : For sending empty commands to the robots" << std::endl
              << "- neither     : For not sending commands at all" << std::endl
              << "- yellow      : For sending commands only to yellow team" << std::endl
              << "- blue        : For sending commands only to blue team" << std::endl
              << "- both        : For sending commands to both teams" << std::endl
              << "- 0..15       : For sending to an individual robot" << std::endl
              << "- all         : For sending to all robots" << std::endl
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

Command askCommand() {
    Command command = Command::INVALID;

    while (command == Command::INVALID) {
        std::cout << "Enter command: ";
        std::string enteredCommand;
        std::getline(std::cin, enteredCommand);
        for (auto& c : enteredCommand) c = toupper(c); // Make command upper case

        if (enteredCommand == "HELP") {
            command = Command::HELP;
        } else if (enteredCommand == "BASESTATION") {       // Basestation
            command = Command::MODE_TO_BASESTATION;
        } else if (enteredCommand == "SIMULATOR") {         // Simulator
            command = Command::MODE_TO_SIMULATOR;
        } else if (enteredCommand == "LEFT") {              // Left
            command = Command::SEND_ROTATE_LEFT_COMMANDS;
        } else if (enteredCommand == "RIGHT") {             // Right
            command = Command::SEND_ROTATE_RIGHT_COMMANDS;
        } else if (enteredCommand == "HALT") {              // Halt
            command = Command::SEND_HALT_COMMANDS;
        } else if (enteredCommand == "NEITHER") {           // Neither
            command = Command::DO_NOT_SEND;
        } else if (enteredCommand == "YELLOW") {            // Yellow
            command = Command::SEND_TO_YELLOW;
        } else if (enteredCommand == "BLUE") {              // Blue
            command = Command::SEND_TO_BLUE;
        } else if (enteredCommand == "BOTH") {              // Both
            command = Command::SEND_TO_BOTH;
        } else if (enteredCommand == "HELP") {              // Help
            command = Command::HELP;
        } else if (stringToRobotId(enteredCommand) >= 0) {
            robotId = stringToRobotId(enteredCommand);
            command = Command::ROBOT_ID;
        } else if (enteredCommand == "ALL") {
            command = Command::ALL_ROBOTS;
        } else if (enteredCommand == "STOP") {
            command = Command::STOP;
        } else {
            std::cout << "Invalid command: '" << enteredCommand << "'. Enter 'help' for more information." << std::endl << std::endl;
        }
    }
    return command;
}

void handleCommand(Command command) {
    switch (command) {
        case Command::HELP: {
            printCommandOptions();
            break;
        } case Command::MODE_TO_BASESTATION: {
            mode = Mode::BASESTATION;
            break;
        } case Command::MODE_TO_SIMULATOR: {
            mode = Mode::SIMULATOR;
            break;
        } case Command::SEND_ROTATE_LEFT_COMMANDS: {
            robotCommand = RobotCommand::LEFT;
            break;
        } case Command::SEND_ROTATE_RIGHT_COMMANDS: {
            robotCommand = RobotCommand::RIGHT;
            break;
        } case Command::SEND_HALT_COMMANDS: {
            robotCommand = RobotCommand::HALT;
            break;
        } case Command::DO_NOT_SEND: {
            team = Team::NEITHER;
            break;
        } case Command::SEND_TO_YELLOW: {
            team = Team::YELLOW;
            break;
        } case Command::SEND_TO_BLUE: {
            team = Team::BLUE;
            break;
        } case Command::SEND_TO_BOTH: {
            team = Team::BOTH;
            break;
        } case Command::ROBOT_ID: {
            sendToAllRobots = false;
            break;
        } case Command::ALL_ROBOTS: {
            sendToAllRobots = true;
            break;
        } case Command::STOP: {
            shouldAskCommands = false;
            break;
        } default: {
            std::cout << "Unknown command" << std::endl;
            break;
        }
    }
}

proto::Setting getSettingsCommand() {
    proto::Setting settings;
    settings.set_serialmode(mode == Mode::BASESTATION);
    return settings;
}

void addHaltCommandForRobot(int id, proto::AICommand& command) {
    auto robotCommand = command.add_commands();
    robotCommand->set_id(id);
    robotCommand->mutable_vel()->set_x(0);
    robotCommand->mutable_vel()->set_y(0);
    robotCommand->set_w(0);
    robotCommand->set_use_angle(false);
    robotCommand->set_dribbler(false);
    robotCommand->set_kicker(false);
    robotCommand->set_chipper(false);
    robotCommand->set_chip_kick_forced(false);
    robotCommand->set_chip_kick_vel(false);
}

void addRotateLeftCommandForRobot(int id, proto::AICommand& command) {
    auto robotCommand = command.add_commands();
    robotCommand->set_id(id);
    robotCommand->mutable_vel()->set_x(0);
    robotCommand->mutable_vel()->set_y(0);
    robotCommand->set_w(ROTATE_SPEED);
    robotCommand->set_use_angle(false);
    robotCommand->set_dribbler(false);
    robotCommand->set_kicker(false);
    robotCommand->set_chipper(false);
    robotCommand->set_chip_kick_forced(false);
    robotCommand->set_chip_kick_vel(false);
}

void addRotateRightCommandForRobot(int id, proto::AICommand& command) {
    auto robotCommand = command.add_commands();
    robotCommand->set_id(id);
    robotCommand->mutable_vel()->set_x(0);
    robotCommand->mutable_vel()->set_y(0);
    robotCommand->set_w(-ROTATE_SPEED);
    robotCommand->set_use_angle(false);
    robotCommand->set_dribbler(false);
    robotCommand->set_kicker(false);
    robotCommand->set_chipper(false);
    robotCommand->set_chip_kick_forced(false);
    robotCommand->set_chip_kick_vel(false);
}

proto::AICommand getCommandToSend() {
    proto::AICommand command;

    if (robotCommand == RobotCommand::HALT) {
        if (sendToAllRobots) {
            for (int id = 0; id < MAX_AMOUNT_OF_ROBOTS; ++id) {
                addHaltCommandForRobot(id, command);
            }
        } else {
            addHaltCommandForRobot(robotId, command);
        }
    } else if (robotCommand == RobotCommand::LEFT) {
        if (sendToAllRobots) {
            for (int id = 0; id < MAX_AMOUNT_OF_ROBOTS; ++id) {
                addRotateLeftCommandForRobot(id, command);
            }
        } else {
            addRotateLeftCommandForRobot(robotId, command);
        }
    } else if (robotCommand == RobotCommand::RIGHT) {
        if (sendToAllRobots) {
            for (int id = 0; id < MAX_AMOUNT_OF_ROBOTS; ++id) {
                addRotateRightCommandForRobot(id, command);
            }
        } else {
            addRotateRightCommandForRobot(robotId, command);
        }
    }
    return command;
}

void runCommands() {
    std::unique_ptr<RobotCommandsYellowPublisher> yellowCommandsPub = std::make_unique<RobotCommandsYellowPublisher>();
    std::unique_ptr<RobotCommandsBluePublisher> blueCommandsPub = std::make_unique<RobotCommandsBluePublisher>();
    std::unique_ptr<SettingsPublisher> settingsPub = std::make_unique<SettingsPublisher>();

    while (shouldRunCommands) {
        settingsPub->publish(getSettingsCommand());

        proto::AICommand commandToSend = getCommandToSend();

        if (team == Team::YELLOW) {
            yellowCommandsPub->publish(commandToSend);
        } else if (team == Team::BLUE) {
            blueCommandsPub->publish(commandToSend);
        } else if (team == Team::BOTH) {
            yellowCommandsPub->publish(commandToSend);
            blueCommandsPub->publish(commandToSend);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
}

std::string getCurrentTargetTeam() {
    switch (team) {
        case Team::NEITHER:
            return "no team";
        case Team::YELLOW:
            return "team yellow";
        case Team::BLUE:
            return "team blue";
        case Team::BOTH:
            return "both teams";
        default:
            return "unknown";
    }
}

std::string getCurrentCommand() {
    switch (robotCommand) {
        case RobotCommand::HALT:
            return "halt";
        case RobotCommand::LEFT:
            return "rotate left";
        case RobotCommand::RIGHT:
            return "rotate right";
        default:
            return "unknown";
    }
}

std::string getCurrentMode() {
    switch (mode) {
        case Mode::BASESTATION:
            return "basestation";
        case Mode::SIMULATOR:
            return "simulator";
        default:
            return "unknown";
    }
}

std::string getRobotTargets() {
    if (sendToAllRobots) {
        return "all robots";
    } else {
        return "robot " + std::to_string(robotId);
    }
}

void printCommandStatus() {
    std::cout << "Sending " << getCurrentCommand() << " to " << getRobotTargets() << " of " << getCurrentTargetTeam() << " via the " << getCurrentMode() << std::endl;
}

int main() {
    std::thread commandRunner(runCommands);

    while (shouldAskCommands) {
        printCommandStatus();
        handleCommand(askCommand());
        std::cout << std::endl;
    }

    shouldRunCommands = false;
    if (commandRunner.joinable()) {
        commandRunner.join();
    }
    return 0;
}