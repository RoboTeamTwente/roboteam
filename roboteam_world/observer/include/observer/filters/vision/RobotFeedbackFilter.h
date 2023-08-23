//
// Created by rolf on 26-05-22.
//

#ifndef RTT_ROBOTFEEDBACKFILTER_H
#define RTT_ROBOTFEEDBACKFILTER_H

#include <map>

#include "observer/filters/vision/robot/RobotPos.h"
#include "proto/RobotProcessedFeedback.pb.h"
#include "roboteam_utils/RobotFeedback.hpp"

class SingleRobotFeedbackFilter {
public:
    explicit SingleRobotFeedbackFilter(const rtt::RobotFeedback& feedback);
    void process(const rtt::RobotFeedback& feedback);
    [[nodiscard]] proto::RobotProcessedFeedback getFilteredFeedback() const;

private:
    rtt::RobotFeedback storedFeedback;
};

class RobotFeedbackFilter {
public:
    void process(const std::vector<rtt::RobotsFeedback>& feedback);

    [[nodiscard]] std::vector<std::pair<TeamRobotID, proto::RobotProcessedFeedback>> getRecentFeedback() const;

private:
    using FeedbackMap = std::map<RobotID, SingleRobotFeedbackFilter>;
    FeedbackMap blueMap;
    FeedbackMap yellowMap;
};


#endif  // RTT_ROBOTFEEDBACKFILTER_H
