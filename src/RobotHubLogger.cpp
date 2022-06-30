#include <RobotHubLogger.hpp>

#include <roboteam_utils/Time.h>
#include <roboteam_utils/Format.hpp>

constexpr char MARPLE_DELIMITER = ',';
constexpr char ROBOT_STATE_MARPLE_HEADER[] = "time,team,xSensAcc1,xSensAcc2,xSensYaw,rateOfTurn,wheelSpeed1,wheelSpeed2,wheelSpeed3,wheelSpeed4";
constexpr char ROBOT_COMMANDS_MARPLE_HEADER[] = "time,team,id,xVel,yVel,targetAngle,targetAngularVel,camAngle,camAngleIsSet,kickSpeed,waitForBall,kickType,dribblerSpeed,ignorePacket";
constexpr char ROBOT_FEEDBACK_MARPLE_HEADER[] = "time,team,source,id,hasBall,ballPos,sensorWorking,velX,velY,angle,xSensCalibrated,capCharged,wheelLocked,wheelBraking,batteryLevel,signalStrength";

namespace rtt {

RobotHubLogger::RobotHubLogger(bool logInMarpleFormat) {
    this->logInMarpleFormat = logInMarpleFormat;

    // Create the loggers
    auto timeString = Time::getDate('-') + "_" + Time::getTime('-');
    auto suffix = logInMarpleFormat ? "_MARPLE.csv" : ".txt";

    this->stateInfoLogger.open("ROBOTSTATES_" + timeString + suffix);
    this->commandsLogger.open("COMMANDS_" + timeString + suffix);
    this->feedbackLogger.open("FEEDBACK_" + timeString + suffix);
    this->infoLogger.open("ROBOTHUB_INFO.txt", std::ios_base::app);

    // Check if all are successfully created
    if (!this->stateInfoLogger.is_open() || !this->commandsLogger.is_open() || !this->feedbackLogger.is_open() || !this->infoLogger.is_open()) {
        throw std::runtime_error("Failed to open file(s) for logging");
    }

    // If we log in marple format, the csv files need a header explaining the values
    if (logInMarpleFormat) {
        // Also add std::fixed to ensure floating point notation is not scientific (aka human readable)
        this->stateInfoLogger << ROBOT_STATE_MARPLE_HEADER << std::fixed << std::endl;
        this->commandsLogger << ROBOT_COMMANDS_MARPLE_HEADER << std::fixed << std::endl;
        this->feedbackLogger << ROBOT_FEEDBACK_MARPLE_HEADER << std::fixed << std::endl;
    }

    // And add a nice divider between errors
    this->infoLogger << std::endl << "Logging started at: '" << timeString << "'." << std::endl;
}

RobotHubLogger::RobotHubLogger(RobotHubLogger &&other) noexcept {
    // First lock all of both mutexes
    std::scoped_lock<std::mutex,
                     std::mutex,
                     std::mutex,
                     std::mutex,
                     std::mutex,
                     std::mutex,
                     std::mutex,
                     std::mutex> lock(this->commandsLogMutex,
                                      this->feedbackLogMutex,
                                      this->stateInfoLogMutex,
                                      this->infoMutex,
                                      other.commandsLogMutex,
                                      other.feedbackLogMutex,
                                      other.stateInfoLogMutex,
                                      other.infoMutex);

    // Then swap them
    std::swap(this->logInMarpleFormat, other.logInMarpleFormat);
    this->commandsLogger.swap(other.commandsLogger);
    this->feedbackLogger.swap(other.feedbackLogger);
    this->stateInfoLogger.swap(other.stateInfoLogger);
    this->infoLogger.swap(other.infoLogger);
}

RobotHubLogger &RobotHubLogger::operator=(RobotHubLogger &&other) noexcept {
    if (this != &other) {
        // First lock all of both mutexes
        std::scoped_lock<std::mutex,
                         std::mutex,
                         std::mutex,
                         std::mutex,
                         std::mutex,
                         std::mutex,
                         std::mutex,
                         std::mutex> lock(this->commandsLogMutex,
                 this->feedbackLogMutex,
                 this->stateInfoLogMutex,
                 this->infoMutex,
                 other.commandsLogMutex,
                 other.feedbackLogMutex,
                 other.stateInfoLogMutex,
                 other.infoMutex);

        // Close all current ofstreams
        this->stateInfoLogger.close();
        this->commandsLogger.close();
        this->feedbackLogger.close();
        this->infoLogger.close();

        // Then swap them
        std::swap(this->logInMarpleFormat, other.logInMarpleFormat);
        this->commandsLogger.swap(other.commandsLogger);
        this->feedbackLogger.swap(other.feedbackLogger);
        this->stateInfoLogger.swap(other.stateInfoLogger);
        this->infoLogger.swap(other.infoLogger);
    }

    return *this;
}

RobotHubLogger::~RobotHubLogger() {
    this->stateInfoLogger.close();
    this->commandsLogger.close();
    this->feedbackLogger.close();
    this->infoLogger.close();
}

int kickTypeToInt(rtt::KickType type) {
    switch (type) {
        case rtt::KickType::NO_KICK:
            return 0;
        case rtt::KickType::KICK:
            return 1;
        case rtt::KickType::CHIP:
            return 2;
        default:
            return -1;
    }
}
int teamToInt(rtt::Team team) {
    switch (team) {
        case Team::YELLOW:
            return 0;
        case Team::BLUE:
            return 1;
        default:
            return -1;
    }
}

int feedbackSourceToInt(rtt::RobotFeedbackSource source) {
    switch (source) {
        case RobotFeedbackSource::SIMULATOR:
            return 0;
        case RobotFeedbackSource::BASESTATION:
            return 1;
        default:
            return -1;
    }
}

void RobotHubLogger::logRobotCommands(const RobotCommands &commands, Team team) {
    std::scoped_lock<std::mutex> lock(this->commandsLogMutex);

    if (this->logInMarpleFormat) {
        auto now = std::chrono::system_clock::now().time_since_epoch().count();
        for (const auto& command : commands) {
            //time,team,id,xVel,yVel,targetAngle,targetAngularVel,camAngle,camAngleIsSet,kickSpeed,waitForBall,kickType,dribblerSpeed,ignorePacket
            this->commandsLogger
                << now << MARPLE_DELIMITER
                << teamToInt(team) << MARPLE_DELIMITER
                << command.id << MARPLE_DELIMITER
                << command.velocity.x << MARPLE_DELIMITER
                << command.velocity.y << MARPLE_DELIMITER
                << command.targetAngle.getValue() << MARPLE_DELIMITER
                << command.targetAngularVelocity << MARPLE_DELIMITER
                << command.cameraAngleOfRobot.getValue() << MARPLE_DELIMITER
                << command.cameraAngleOfRobotIsSet << MARPLE_DELIMITER
                << command.kickSpeed << MARPLE_DELIMITER
                << command.waitForBall << MARPLE_DELIMITER
                << kickTypeToInt(command.kickType) << MARPLE_DELIMITER
                << command.dribblerSpeed << MARPLE_DELIMITER
                << command.ignorePacket << std::endl;
        }
    } else {
        std::string teamStr = teamToString(team);
        std::string timeStr = Time::getTimeWithMilliseconds(':');

        for (const auto& command : commands) {
            this->commandsLogger << "[" << timeStr << ", " << teamStr << "] " << command << std::endl;
        }
    }
}

void RobotHubLogger::logRobotStateInfo(const REM_RobotStateInfo &info, Team team) {
    std::scoped_lock<std::mutex> lock(this->stateInfoLogMutex);
    if (this->logInMarpleFormat) {
        auto now = std::chrono::system_clock::now().time_since_epoch().count();
        //time,team,xSensAcc1,xSensAcc2,xSensYaw,rateOfTurn,wheelSpeed1,wheelSpeed2,wheelSpeed3,wheelSpeed4
        this->stateInfoLogger
            << now << MARPLE_DELIMITER
            << teamToInt(team) << MARPLE_DELIMITER
            << info.xsensAcc1 << MARPLE_DELIMITER
            << info.xsensAcc2 << MARPLE_DELIMITER
            << info.xsensYaw << MARPLE_DELIMITER
            << info.rateOfTurn << MARPLE_DELIMITER
            << info.wheelSpeed1 << MARPLE_DELIMITER
            << info.wheelSpeed2 << MARPLE_DELIMITER
            << info.wheelSpeed3 << MARPLE_DELIMITER
            << info.wheelSpeed4 << std::endl;
    } else {
        this->stateInfoLogger
            << "[" << Time::getTimeWithMilliseconds(':') << "] "
            << "Team: " << teamToString(team)
            << "Id: " << formatString("%2i", info.id) << ", "
            << "MsgId: " << formatString("%5i", info.messageId) << ", "
            << "xSensAcc1: " << formatString("%7f", info.xsensAcc1) << ", "
            << "xSensAcc2: " << formatString("%7f", info.xsensAcc2) << ", "
            << "xSensYaw: " << formatString("%7f", info.xsensYaw) << ", "
            << "rateOfTurn: " << formatString("%7f", info.rateOfTurn) << ", "
            << "wheelSp1: " << formatString("%&7f", info.wheelSpeed1) << ", "
            << "wheelSp2: " << formatString("%7f", info.wheelSpeed2) << ", "
            << "wheelSp3: " << formatString("%7f", info.wheelSpeed3) << ", "
            << "wheelSp4: " << formatString("%7f", info.wheelSpeed4) << std::endl;
    }
}

void RobotHubLogger::logRobotFeedback(const RobotsFeedback &feedback) {
    std::scoped_lock<std::mutex> lock(this->feedbackLogMutex);

    if (this->logInMarpleFormat) {
        auto now = std::chrono::system_clock::now().time_since_epoch().count();
        auto team = teamToInt(feedback.team);
        auto source = feedbackSourceToInt(feedback.source);

        for (const auto& robot : feedback.feedback) {
            //time,team,source,id,hasBall,ballPos,sensorWorking,velX,velY,angle,xSensCalibrated,capCharged,wheelLocked,wheelBraking,batteryLevel,signalStrength
            this->feedbackLogger
                << now << MARPLE_DELIMITER
                << team << MARPLE_DELIMITER
                << source << MARPLE_DELIMITER
                << robot.id << MARPLE_DELIMITER
                << robot.hasBall << MARPLE_DELIMITER
                << robot.ballPosition << MARPLE_DELIMITER
                << robot.ballSensorIsWorking << MARPLE_DELIMITER
                << robot.velocity.x << MARPLE_DELIMITER
                << robot.velocity.y << MARPLE_DELIMITER
                << robot.angle.getValue() << MARPLE_DELIMITER
                << robot.xSensIsCalibrated << MARPLE_DELIMITER
                << robot.capacitorIsCharged << MARPLE_DELIMITER
                << robot.wheelLocked << MARPLE_DELIMITER
                << robot.wheelBraking << MARPLE_DELIMITER
                << robot.batteryLevel << MARPLE_DELIMITER
                << robot.signalStrength << std::endl;
        }
    } else {
        std::string teamStr = teamToString(feedback.team);
        std::string sourceStr = robotFeedbackSourceToString(feedback.source);
        std::string timeStr = Time::getTimeWithMilliseconds(':');

        for (const auto &robot : feedback.feedback) {
            this->feedbackLogger << "[" << timeStr << ", " << teamStr << ", " << sourceStr << "] " << robot << std::endl;
        }
    }
}

void RobotHubLogger::logInfo(const std::string &infoMessage) {
    std::scoped_lock<std::mutex> lock(this->infoMutex);

    auto time = Time::getTimeWithMilliseconds(':');
    this->infoLogger << "[" << time << "] " << infoMessage << std::endl;
}

} // namespace rtt
