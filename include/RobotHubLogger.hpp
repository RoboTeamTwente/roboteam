#pragma once

#include <fstream>
#include <mutex>
#include <roboteam_utils/RobotCommands.hpp>
#include <roboteam_utils/RobotFeedback.hpp>
#include <REM_RobotStateInfo.h>

namespace rtt {

// This class is meant to contain all the thread-safe logging functionality of RobotHub
class RobotHubLogger {
public:
    // Marple format is csv format. False for "human-readable" text file
    // Only moving is allowed
    explicit RobotHubLogger(bool logInMarpleFormat);
    RobotHubLogger(const RobotHubLogger& other) = delete;
    RobotHubLogger(RobotHubLogger&& other) noexcept;
    RobotHubLogger& operator=(const RobotHubLogger& other) = delete;
    RobotHubLogger& operator=(RobotHubLogger&& other) noexcept;
    ~RobotHubLogger();

    void logRobotCommands(const RobotCommands&, Team);
    void logRobotFeedback(const RobotsFeedback&);
    void logRobotStateInfo(const REM_RobotStateInfo&, Team);
    void logInfo(const std::string& errorMessage);

private:
    bool logInMarpleFormat;

    std::mutex commandsLogMutex;
    std::mutex feedbackLogMutex;
    std::mutex stateInfoLogMutex;
    std::mutex infoMutex;

    std::ofstream commandsLogger;
    std::ofstream feedbackLogger;
    std::ofstream stateInfoLogger;
    std::ofstream infoLogger;
};

} // namespace rtt