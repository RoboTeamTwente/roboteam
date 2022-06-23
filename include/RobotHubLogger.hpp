#pragma once

#include <fstream>
#include <mutex>
#include <roboteam_utils/RobotCommands.hpp>
#include <roboteam_utils/RobotFeedback.hpp>
#include <REM_RobotStateInfo.h>

namespace rtt {

// This class is meant to contain all the logging functionality of RobotHub
class RobotHubLogger {
public:
    // Marple format is csv format. False for "human readable"
    explicit RobotHubLogger(bool logInMarpleFormat);
    ~RobotHubLogger();

    void logRobotCommands(const RobotCommands&, Team);
    void logRobotFeedback(const RobotsFeedback&);
    void logRobotStateInfo(const REM_RobotStateInfo&, Team);

private:
    bool logInMarpleFormat;

    std::ofstream commandsLogger;
    std::ofstream feedbackLogger;
    std::ofstream stateInfoLogger;
};

} // namespace rtt