//
// Created by rolf on 29-10-21.
//

#include "observer/filters/RobotFeedbackFilter.h"

void RobotFeedbackFilter::process(const proto::RobotData& data) {
    auto& map = data.isyellow() ? lastYellowFeedback : lastBlueFeedback;
    map.clear();
    for (const auto& feedback : data.receivedfeedback()) {
        map[feedback.id()] = feedback;
    }
}

std::vector<proto::RobotFeedback> RobotFeedbackFilter::getData(bool teamIsYellow) const {
    std::vector<proto::RobotFeedback> feedback;
    const auto& map = teamIsYellow ? lastYellowFeedback : lastBlueFeedback;
    feedback.reserve(map.size());
    for (const auto& elem : map) {
        feedback.emplace_back(elem.second);
    }
    return feedback;
}
