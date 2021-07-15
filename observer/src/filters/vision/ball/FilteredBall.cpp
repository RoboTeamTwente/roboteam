//
// Created by rolf on 05-07-21.
//

#include "filters/vision/ball/FilteredBall.h"
proto::WorldBall FilteredBall::asWorldBall() const {
  proto::WorldBall proto_ball;
  proto_ball.mutable_vel()->set_x(velocity.x());
  proto_ball.mutable_vel()->set_y(velocity.y());
  proto_ball.mutable_pos()->set_x(position.x());
  proto_ball.mutable_pos()->set_y(position.y());


  //TODO: fix the below fields
  static constexpr double BALL_RADIUS = 0.021333;
  proto_ball.set_z(BALL_RADIUS);
  proto_ball.set_z_vel(0.0);
  proto_ball.set_visible(true);
  proto_ball.set_area(0);

  return proto_ball;
}
