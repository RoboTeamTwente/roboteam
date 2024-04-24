#pragma once

#include <RobotHubMode.h>

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

    basestation::BasestationManagerStatus basestationManagerStatus{};

    rtt::net::RobotHubMode robotHubMode;
    mutable std::mutex robotHubModeMutex;  // Mutex to protect the robotHubMode

    void unlockRobotStatsMutex() { robotStatsMutex.unlock();}
    void lockRobotStatsMutex() { robotStatsMutex.lock();}

    std::size_t yellowTeamBytesSent;
    std::size_t blueTeamBytesSent;
    std::size_t feedbackBytesSent;
    int yellowTeamPacketsDropped;
    int blueTeamPacketsDropped;
    int feedbackPacketsDropped;

    void incrementCommandsReceivedCounter(int id, rtt::Team color);
    void incrementFeedbackReceivedCounter(int id, rtt::Team color);

    void print() const;

   private:
    std::chrono::time_point<std::chrono::steady_clock> startTime;
    mutable std::mutex robotStatsMutex; // Mutex to protect the robotStats
    std::array<int, MAX_ROBOT_STATISTICS> yellowCommandsSent{};
    std::array<int, MAX_ROBOT_STATISTICS> yellowFeedbackReceived{};

    std::array<int, MAX_ROBOT_STATISTICS> blueCommandsSent{};
    std::array<int, MAX_ROBOT_STATISTICS> blueFeedbackReceived{};

    [[nodiscard]] std::string getRobotStats(int id, rtt::Team team) const;
    [[nodiscard]] std::string getRunTime() const;
    [[nodiscard]] std::string getRobotHubMode() const;
    [[nodiscard]] std::string getAmountOfBasestations() const;
    [[nodiscard]] std::string getWantedBasestations() const;
    [[nodiscard]] std::string getSelectedBasestations() const;

    [[nodiscard]] std::string numberToSideBox(int n) const;

    static std::string wantedBasestationsToString(basestation::WantedBasestations wantedBasestations);
};

}  // namespace rtt::robothub