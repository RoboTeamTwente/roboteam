//
// Created by rolf on 29-10-21.
//

#include "observer/filters/RobotFeedbackFilter.h"

void RobotFeedbackFilter::process(const rtt::RobotsFeedback& data) {
    auto& map = data.team == rtt::Team::YELLOW ? lastYellowFeedback : lastBlueFeedback;
    map.clear();
    for (const auto& feedback : data.feedback) {
        map[feedback.id] = feedback;
    }
}

std::vector<rtt::RobotFeedback> RobotFeedbackFilter::getData(bool teamIsYellow) const {
    std::vector<rtt::RobotFeedback> feedback;
    const auto& map = teamIsYellow ? lastYellowFeedback : lastBlueFeedback;
    feedback.reserve(map.size());
    for (const auto& elem : map) {
        feedback.emplace_back(elem.second);
    }
    return feedback;
}
