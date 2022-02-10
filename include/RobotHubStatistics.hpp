#pragma once

#include <basestation/BasestationManager.hpp>
#include <utilities.h>
#include <string>
#include <chrono>
#include <array>

namespace rtt::robothub {

constexpr int MAX_ROBOT_STATISTICS = 16;

class RobotHubStatistics {
public:
    RobotHubStatistics();

    void resetValues();

    basestation::BasestationManagerStatus basestationManagerStatus;

    utils::RobotHubMode robotHubMode;

    int yellowTeamBytesSent;
    int blueTeamBytesSent;
    int feedbackBytesSent;
    int yellowTeamPacketsDropped;
    int blueTeamPacketsDropped;
    int feedbackPacketsDropped;

    void incrementCommandsReceivedCounter(int id, utils::TeamColor color);
    void incrementFeedbackReceivedCounter(int id, utils::TeamColor color);

    void print();
private:
    std::chrono::time_point<std::chrono::steady_clock> startTime;

    std::array<int, MAX_ROBOT_STATISTICS> yellowCommandsSent;
    std::array<int, MAX_ROBOT_STATISTICS> yellowFeedbackReceived;
    
    std::array<int, MAX_ROBOT_STATISTICS> blueCommandsSent;
    std::array<int, MAX_ROBOT_STATISTICS> blueFeedbackReceived;
    
    std::string getRobotStats(int id, utils::TeamColor team) const;
    std::string getRunTime();
    std::string getRobotHubMode();
    std::string getAmountOfBasestations();
    std::string getWantedBasestations();
    std::string getSelectedBasestations();

    std::string numberToSideBox(int n);

    template<typename ... Args>
    static std::string string_format( const std::string& format, Args ... args );

    static std::string wantedBasestationsToString(basestation::WantedBasestations wantedBasestations);
};

} // namespace rtt::robothub