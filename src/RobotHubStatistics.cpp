#include <roboteam_utils/Print.h>
#include <roboteam_utils/Format.hpp>

#include <RobotHubStatistics.hpp>
#include <chrono>
#include <sstream>

namespace rtt::robothub {

RobotHubStatistics::RobotHubStatistics() {
    this->startTime = std::chrono::steady_clock::now();

    // Fill arrays with 0
    this->yellowCommandsSent.fill(0);
    this->yellowFeedbackReceived.fill(0);
    this->blueCommandsSent.fill(0);
    this->blueFeedbackReceived.fill(0);

    this->yellowTeamBytesSent = 0;
    this->blueTeamBytesSent = 0;
    this->feedbackBytesSent = 0;
    this->yellowTeamPacketsDropped = 0;
    this->blueTeamPacketsDropped = 0;
    this->feedbackPacketsDropped = 0;
}

void RobotHubStatistics::resetValues() {
    this->yellowCommandsSent.fill(0);
    this->yellowFeedbackReceived.fill(0);
    this->blueCommandsSent.fill(0);
    this->blueFeedbackReceived.fill(0);

    this->yellowTeamBytesSent = 0;
    this->blueTeamBytesSent = 0;
    this->feedbackBytesSent = 0;
    this->yellowTeamPacketsDropped = 0;
    this->blueTeamPacketsDropped = 0;
    this->feedbackPacketsDropped = 0;
}

void RobotHubStatistics::print() const {
    std::array<std::string, MAX_ROBOT_STATISTICS> y;  // Contains the strings for yellow robots
    std::array<std::string, MAX_ROBOT_STATISTICS> b;  // Contains the strings for blue robots

    for (int i = 0; i < MAX_ROBOT_STATISTICS; ++i) {
        y[i] = getRobotStats(i, rtt::Team::YELLOW);
        b[i] = getRobotStats(i, rtt::Team::BLUE);
    }

    std::stringstream ss;

    ss << "                          ┏━━━━━━━━━━━━━━━━━━━┓                           " << std::endl
       << "┏━━━━━━━━━━━━━━━━━━━━━━━━━┫ Roboteam RobotHub ┣━━━┳━━━━━━━━━━━━━━━━━━━━━━┓" << std::endl
       << "┃                         ┗━━━━━━━━━━━━━━━━━━━┛   ┃                      ┃" << std::endl
       << "┃ Mode: " << this->getRobotHubMode() << "          " << this->getRunTime() << " ┃     Basestations     ┃" << std::endl
       << "┃                                                 ┃ Connected: " << this->getAmountOfBasestations() << " ┃" << std::endl
       << "┃ Incoming data: (id: commands, feedbacks)        ┃ Wanted:    " << this->getWantedBasestations() << " ┃" << std::endl
       << "┃ Yellow team:                                    ┃ Selected:  " << this->getSelectedBasestations() << " ┃" << std::endl
       << "┃" << y[0] << " │" << y[4] << " │ " << y[8] << " │ " << y[12] << " ┣━━━━━━━━━━━━━━━━━━━━━━┫" << std::endl
       << "┃" << y[1] << " │" << y[5] << " │ " << y[9] << " │ " << y[13] << " ┃      Bytes sent      ┃" << std::endl
       << "┃" << y[2] << " │" << y[6] << " │ " << y[10] << " │ " << y[14] << " ┃ Yellow team: " << this->numberToSideBox(this->yellowTeamBytesSent) << " ┃" << std::endl
       << "┃" << y[3] << " │" << y[7] << " │ " << y[11] << " │ " << y[15] << " ┃ Blue team:   " << this->numberToSideBox(this->blueTeamBytesSent) << " ┃" << std::endl
       << "┃                                                 ┃ Feedback:    " << this->numberToSideBox(this->feedbackBytesSent) << " ┃" << std::endl
       << "┃ Blue team:                                      ┣━━━━━━━━━━━━━━━━━━━━━━┫" << std::endl
       << "┃" << b[0] << " │" << b[4] << " │ " << b[8] << " │ " << b[12] << " ┃    Packets dropped   ┃" << std::endl
       << "┃" << b[1] << " │" << b[5] << " │ " << b[9] << " │ " << b[13] << " ┃ Yellow team: " << this->numberToSideBox(this->yellowTeamPacketsDropped) << " ┃" << std::endl
       << "┃" << b[2] << " │" << b[6] << " │ " << b[10] << " │ " << b[14] << " ┃ Blue team:   " << this->numberToSideBox(this->blueTeamPacketsDropped) << " ┃" << std::endl
       << "┃" << b[3] << " │" << b[7] << " │ " << b[11] << " │ " << b[15] << " ┃ Feedback:    " << this->numberToSideBox(this->feedbackPacketsDropped) << " ┃" << std::endl
       << "┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┻━━━━━━━━━━━━━━━━━━━━━━┛" << std::endl;

    RTT_INFO("\n", ss.str())
}

void RobotHubStatistics::incrementCommandsReceivedCounter(int id, rtt::Team color) {
    auto& arrayToIncrement = color == rtt::Team::YELLOW ? this->yellowCommandsSent : this->blueCommandsSent;
    arrayToIncrement[id]++;
}
void RobotHubStatistics::incrementFeedbackReceivedCounter(int id, rtt::Team color) {
    auto& arrayToIncrement = color == rtt::Team::YELLOW ? this->yellowFeedbackReceived : this->blueFeedbackReceived;
    arrayToIncrement[id]++;
}

std::string RobotHubStatistics::getRobotStats(int robotId, rtt::Team team) const {
    std::string stats;

    switch (team) {
        case rtt::Team::BLUE:
            stats = formatString("%2d: %2d, %2d", robotId, this->blueCommandsSent[robotId], this->blueFeedbackReceived[robotId]);
            break;
        case rtt::Team::YELLOW:
            stats = formatString("%2d: %2d, %2d", robotId, this->yellowCommandsSent[robotId], this->yellowFeedbackReceived[robotId]);
            break;
        default:
            stats = formatString("%2d:  ?,  ?", robotId);
            break;
    }

    return stats;
}

std::string RobotHubStatistics::getRunTime() const {
    auto now = std::chrono::steady_clock::now();
    int64_t runTime = std::chrono::duration_cast<std::chrono::seconds>(now - this->startTime).count();

    int hours = (int)(runTime / 3600);
    int minutes = (int)((runTime % 3600) / 60);
    int seconds = (int)(runTime % 60);

    std::string time;

    if (hours > 0) {
        time = formatString("Running: %2dh %2dm %2ds", hours, minutes, seconds);
    } else if (minutes > 0) {
        time = formatString("Running: %2dm %2ds", minutes, seconds);
    } else {
        time = formatString("Running: %2ds", seconds);
    }

    std::string text = formatString("%20s", time.c_str());

    return text;
}

std::string RobotHubStatistics::getRobotHubMode() const {
    std::string mode = utils::modeToString(this->robotHubMode);
    std::string text = formatString("%-11s", mode.c_str());
    return text;
}

std::string RobotHubStatistics::getAmountOfBasestations() const {
    return formatString("%-9d", this->basestationManagerStatus.basestationCollection.amountOfBasestations);
}

std::string RobotHubStatistics::getWantedBasestations() const {
    std::string basestations = wantedBasestationsToString(this->basestationManagerStatus.basestationCollection.wantedBasestations);
    return formatString("%-9s", basestations.c_str());
}
std::string RobotHubStatistics::getSelectedBasestations() const {
    std::string basestations;

    if (this->basestationManagerStatus.basestationCollection.hasYellowBasestation && this->basestationManagerStatus.basestationCollection.hasBlueBasestation) {
        basestations = wantedBasestationsToString(basestation::WantedBasestations::YELLOW_AND_BLUE);
    } else if (this->basestationManagerStatus.basestationCollection.hasYellowBasestation) {
        basestations = wantedBasestationsToString(basestation::WantedBasestations::ONLY_YELLOW);
    } else if (this->basestationManagerStatus.basestationCollection.hasBlueBasestation) {
        basestations = wantedBasestationsToString(basestation::WantedBasestations::ONLY_BLUE);
    } else {
        basestations = wantedBasestationsToString(basestation::WantedBasestations::NEITHER_YELLOW_NOR_BLUE);
    }

    return formatString("%-9s", basestations.c_str());
}
std::string RobotHubStatistics::numberToSideBox(int n) const { return formatString("%7d", n); }

std::string RobotHubStatistics::wantedBasestationsToString(basestation::WantedBasestations wantedBasestations) {
    std::string text;
    switch (wantedBasestations) {
        case basestation::WantedBasestations::ONLY_YELLOW:
            text = "Yellow";
            break;
        case basestation::WantedBasestations::ONLY_BLUE:
            text = "Blue";
            break;
        case basestation::WantedBasestations::YELLOW_AND_BLUE:
            text = "Both";
            break;
        case basestation::WantedBasestations::NEITHER_YELLOW_NOR_BLUE:
            text = "Neither";
            break;
        default:
            text = "Unknown";
            break;
    }
    return text;
}

}  // namespace rtt::robothub