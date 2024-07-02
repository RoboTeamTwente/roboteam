#include "filters/vision/ball/FilteredBall.h"

#include <utility>
proto::WorldBall FilteredBall::asWorldBall() const {
    proto::WorldBall proto_ball;
    proto_ball.mutable_vel()->set_x(velocity.x());
    proto_ball.mutable_vel()->set_y(velocity.y());
    proto_ball.mutable_pos()->set_x(position.x());
    proto_ball.mutable_pos()->set_y(position.y());

    // TODO: fix the below fields
    static constexpr double BALL_RADIUS = 0.021333;
    proto_ball.set_z(BALL_RADIUS);
    proto_ball.set_z_vel(0.0);
    proto_ball.set_visible(true);
    proto_ball.set_area(0);

    return proto_ball;
}
FilteredBall::FilteredBall(Eigen::Vector2d pos, Eigen::Vector2d vel, Time time, Eigen::Vector2d positionCamera, std::optional<BallObservation> currentObservation)
    : position(std::move(pos)), velocity(std::move(vel)), time(time), positionCamera(std::move(positionCamera)), currentObservation(std::move(currentObservation)) {}