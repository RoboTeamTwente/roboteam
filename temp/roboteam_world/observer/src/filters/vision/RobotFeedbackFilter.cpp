//
// Created by rolf on 26-05-22.
//

#include "filters/vision/RobotFeedbackFilter.h"

void RobotFeedbackFilter::process(const std::vector<rtt::RobotsFeedback> &feedback) {
    for(const auto& message : feedback){
        auto& map = message.team == rtt::Team::YELLOW ? yellowMap : blueMap;

        for(const auto& robotFeedback : message.feedback) {
            RobotID id(robotFeedback.id);
            auto iterator = map.find(id);
            if(iterator == map.end()){
                map.emplace(id,robotFeedback);
            }else{
                iterator->second.process(robotFeedback);
            }
        }
    }

}

std::vector<std::pair<TeamRobotID,proto::RobotProcessedFeedback>> RobotFeedbackFilter::getRecentFeedback() const{
    std::vector<std::pair<TeamRobotID,proto::RobotProcessedFeedback>> feedbackVec;
    for (const auto& [id,robot] : blueMap){
        feedbackVec.emplace_back(TeamRobotID(id.robotID,TeamColor::BLUE),robot.getFilteredFeedback());
    }
    for (const auto& [id,robot] : yellowMap){
        feedbackVec.emplace_back(TeamRobotID(id.robotID,TeamColor::YELLOW),robot.getFilteredFeedback());
    }
    return feedbackVec;
}

//For now very simple, simply stores feedback
SingleRobotFeedbackFilter::SingleRobotFeedbackFilter(const rtt::RobotFeedback &feedback) : storedFeedback{feedback}{

}

void SingleRobotFeedbackFilter::process(const rtt::RobotFeedback &feedback) {
    storedFeedback = feedback;
}


//TODO: now just returns last received feedback: use timestamps and some cutoff limit to prevent sending old feedback
proto::RobotProcessedFeedback SingleRobotFeedbackFilter::getFilteredFeedback() const {
    proto::RobotProcessedFeedback feedback;
    feedback.set_ball_sensor_sees_ball(storedFeedback.ballSensorSeesBall);
    feedback.set_ball_position(storedFeedback.ballPosition);
    feedback.set_ball_sensor_is_working(storedFeedback.ballSensorIsWorking);
    feedback.set_dribbler_sees_ball(storedFeedback.dribblerSeesBall);
    feedback.set_battery_level(storedFeedback.batteryLevel);
    feedback.set_xsens_is_calibrated(storedFeedback.xSensIsCalibrated);
    feedback.set_signal_strength(storedFeedback.signalStrength);
    feedback.set_capacitor_is_charged(storedFeedback.capacitorIsCharged);

    //TODO: wheel ordering? which bit corresponds to which wheel?
    feedback.mutable_wheelinformation()->mutable_rightfront()->set_braking(storedFeedback.wheelBraking & 1);
    feedback.mutable_wheelinformation()->mutable_rightfront()->set_locked(storedFeedback.wheelLocked & 1);

    feedback.mutable_wheelinformation()->mutable_rightback()->set_braking(storedFeedback.wheelBraking & 2);
    feedback.mutable_wheelinformation()->mutable_rightback()->set_locked(storedFeedback.wheelLocked & 2);

    feedback.mutable_wheelinformation()->mutable_leftback()->set_braking(storedFeedback.wheelBraking & 4);
    feedback.mutable_wheelinformation()->mutable_leftback()->set_locked(storedFeedback.wheelLocked & 4);

    feedback.mutable_wheelinformation()->mutable_leftfront()->set_braking(storedFeedback.wheelBraking & 8);
    feedback.mutable_wheelinformation()->mutable_leftfront()->set_locked(storedFeedback.wheelLocked & 8);

    return feedback;
}
