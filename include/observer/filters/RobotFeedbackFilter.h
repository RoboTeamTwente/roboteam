//
// Created by rolf on 29-10-21.
//

#ifndef RTT_ROBOTFEEDBACKFILTER_H
#define RTT_ROBOTFEEDBACKFILTER_H

#include <map>
#include <roboteam_proto/RobotFeedback.pb.h>
#include <roboteam_proto/RobotData.pb.h>


class RobotFeedbackFilter {
private:
    std::map<unsigned int,proto::RobotFeedback> lastBlueFeedback;
    std::map<unsigned int,proto::RobotFeedback> lastYellowFeedback;
public:
    void process(const proto::RobotData& data);
    std::vector<proto::RobotFeedback> getData(bool teamIsYellow) const;
};


#endif //RTT_ROBOTFEEDBACKFILTER_H
