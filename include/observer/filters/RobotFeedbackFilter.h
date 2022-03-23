//
// Created by rolf on 29-10-21.
//

#ifndef RTT_ROBOTFEEDBACKFILTER_H
#define RTT_ROBOTFEEDBACKFILTER_H

#include <roboteam_utils/RobotFeedback.hpp>

#include <map>

class RobotFeedbackFilter {
   private:
    std::map<unsigned int, rtt::RobotFeedback> lastBlueFeedback;
    std::map<unsigned int, rtt::RobotFeedback> lastYellowFeedback;

   public:
    void process(const rtt::RobotsFeedback& data);
    std::vector<rtt::RobotFeedback> getData(bool teamIsYellow) const;
};

#endif  // RTT_ROBOTFEEDBACKFILTER_H
