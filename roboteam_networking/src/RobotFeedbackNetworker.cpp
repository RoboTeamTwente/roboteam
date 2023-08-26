#include <RobotFeedbackNetworker.hpp>

namespace rtt::net {

proto::RobotFeedbackSource feedbackSourceToProto(rtt::RobotFeedbackSource source) {
    switch (source) {
        case rtt::RobotFeedbackSource::SIMULATOR:
            return proto::RobotFeedbackSource::SIMULATOR;
        case rtt::RobotFeedbackSource::BASESTATION:
            return proto::RobotFeedbackSource::BASESTATION;
        default:
            return proto::RobotFeedbackSource::SIMULATOR;
    }
}

rtt::RobotFeedbackSource protoFeedbackSourceToSource(const proto::RobotFeedbackSource& protoSource) {
    switch (protoSource) {
        case proto::RobotFeedbackSource::SIMULATOR:
            return rtt::RobotFeedbackSource::SIMULATOR;
        case proto::RobotFeedbackSource::BASESTATION:
            return rtt::RobotFeedbackSource::BASESTATION;
        default:
            return rtt::RobotFeedbackSource::SIMULATOR;
    }
}

proto::RobotTeam robotTeamToProto(rtt::Team team) {
    switch (team) {
        case rtt::Team::YELLOW:
            return proto::RobotTeam::YELLOW_TEAM;
        case rtt::Team::BLUE:
            return proto::RobotTeam::BLUE_TEAM;
        default:
            return proto::RobotTeam::YELLOW_TEAM;
    }
}

rtt::Team protoTeamToTeam(const proto::RobotTeam& team) {
    switch (team) {
        case proto::RobotTeam::YELLOW_TEAM:
            return rtt::Team::YELLOW;
        case proto::RobotTeam::BLUE_TEAM:
            return rtt::Team::BLUE;
        default:
            return rtt::Team::YELLOW;
    }
}

proto::RobotsFeedback feedbackToProto(const rtt::RobotsFeedback& robotsFeedback) {
    proto::RobotsFeedback protoFeedback;
    protoFeedback.set_team(robotTeamToProto(robotsFeedback.team));
    protoFeedback.set_source(feedbackSourceToProto(robotsFeedback.source));

    for (const auto& feedback : robotsFeedback.feedback) {
        auto protoRobot = protoFeedback.add_robots_feedback();
        protoRobot->set_id(feedback.id);
        protoRobot->set_ball_sensor_sees_ball(feedback.ballSensorSeesBall);
        protoRobot->set_ball_position(feedback.ballPosition);
        protoRobot->set_ball_sensor_is_working(feedback.ballSensorIsWorking);
        protoRobot->set_dribbler_sees_ball(feedback.dribblerSeesBall);
        protoRobot->set_estimated_velocity_x(feedback.velocity.x);
        protoRobot->set_estimated_velocity_y(feedback.velocity.y);
        protoRobot->set_estimated_angle(feedback.angle.getValue());
        protoRobot->set_xsens_is_calibrated(feedback.xSensIsCalibrated);
        protoRobot->set_capacitor_is_charged(feedback.capacitorIsCharged);
        protoRobot->set_wheels_locked(feedback.wheelLocked);
        protoRobot->set_wheels_braking(feedback.wheelBraking);
        protoRobot->set_battery_level(feedback.batteryLevel);
        protoRobot->set_signal_strength(feedback.signalStrength);
    }

    return protoFeedback;
}

rtt::RobotsFeedback protoFeedbackToRobotsFeedback(const proto::RobotsFeedback& protoFeedbacks) {
    rtt::RobotsFeedback robotsFeedback;
    robotsFeedback.team = protoTeamToTeam(protoFeedbacks.team());
    robotsFeedback.source = protoFeedbackSourceToSource(protoFeedbacks.source());

    for (const auto& protoFeedback : protoFeedbacks.robots_feedback()) {
        rtt::RobotFeedback feedback = {.id = protoFeedback.id(),
                                       .ballSensorSeesBall = protoFeedback.ball_sensor_sees_ball(),
                                       .ballPosition = protoFeedback.ball_position(),
                                       .ballSensorIsWorking = protoFeedback.ball_sensor_is_working(),
                                       .dribblerSeesBall = protoFeedback.dribbler_sees_ball(),
                                       .velocity = Vector2(protoFeedback.estimated_velocity_x(), protoFeedback.estimated_velocity_y()),
                                       .angle = Angle(protoFeedback.estimated_angle()),
                                       .xSensIsCalibrated = protoFeedback.xsens_is_calibrated(),
                                       .capacitorIsCharged = protoFeedback.capacitor_is_charged(),
                                       .wheelLocked = protoFeedback.wheels_locked(),
                                       .wheelBraking = protoFeedback.wheels_braking(),
                                       .batteryLevel = protoFeedback.battery_level(),
                                       .signalStrength = protoFeedback.signal_strength()};
        robotsFeedback.feedback.push_back(feedback);
    }

    return robotsFeedback;
}

RobotFeedbackPublisher::RobotFeedbackPublisher() : utils::Publisher(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL) {}

std::size_t RobotFeedbackPublisher::publish(const rtt::RobotsFeedback& feedback) {
    auto protoRobotsFeedback = feedbackToProto(feedback);
    return this->send(protoRobotsFeedback.SerializeAsString());
}

RobotFeedbackSubscriber::RobotFeedbackSubscriber(const std::function<void(const rtt::RobotsFeedback&)>& callback)
    : utils::Subscriber(utils::ChannelType::ROBOT_FEEDBACK_CHANNEL, [&](const std::string& message) { this->onPublishedMessage(message); }), callback(callback) {
    if (callback == nullptr) {
        throw utils::InvalidCallbackException("Callback was nullptr");
    }
}

void RobotFeedbackSubscriber::onPublishedMessage(const std::string& message) {
    proto::RobotsFeedback feedback;
    feedback.ParseFromString(message);
    this->callback(protoFeedbackToRobotsFeedback(feedback));
}

}  // namespace rtt::net