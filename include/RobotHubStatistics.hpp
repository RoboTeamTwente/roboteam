#pragma once

#include <utilities.h>

#include <array>
#include <basestation/BasestationManager.hpp>
#include <chrono>
#include <string>

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

    void incrementCommandsReceivedCounter(int id, rtt::Team color);
    void incrementFeedbackReceivedCounter(int id, rtt::Team color);

    void print() const;

   private:
    std::chrono::time_point<std::chrono::steady_clock> startTime;

    std::array<int, MAX_ROBOT_STATISTICS> yellowCommandsSent;
    std::array<int, MAX_ROBOT_STATISTICS> yellowFeedbackReceived;

    std::array<int, MAX_ROBOT_STATISTICS> blueCommandsSent;
    std::array<int, MAX_ROBOT_STATISTICS> blueFeedbackReceived;

    std::string getRobotStats(int id, rtt::Team team) const;
    std::string getRunTime() const;
    std::string getRobotHubMode() const;
    std::string getAmountOfBasestations() const;
    std::string getWantedBasestations() const;
    std::string getSelectedBasestations() const;

    std::string numberToSideBox(int n) const;

    template <typename... Args>
    static std::string string_format(const std::string& format, Args... args);

    static std::string wantedBasestationsToString(basestation::WantedBasestations wantedBasestations);
};

}  // namespace rtt::robothub