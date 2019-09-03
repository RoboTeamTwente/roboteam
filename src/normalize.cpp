#include "normalize.h"

namespace roboteam_utils {

float mm_to_m(float scalar) {
  return scalar/1000;
}

// rotate a ball
void rotate(roboteam_proto::WorldBall *ball) {
  ball->mutable_pos()->set_x(ball->pos().x()*-1);
  ball->mutable_pos()->set_y(ball->pos().y()*-1);
}

// rotate a single robot
void rotate(roboteam_proto::WorldRobot *robot) {
  robot->mutable_pos()->set_x(robot->pos().x()*-1);
  robot->mutable_pos()->set_y(robot->pos().y()*-1);
  robot->mutable_vel()->set_x(robot->vel().x()*-1);
  robot->mutable_vel()->set_y(robot->vel().y()*-1);
  robot->set_w(robot->w()*-1);
  robot->set_angle(static_cast<float>(rtt::cleanAngle(robot->angle() + M_PI)));
}

// rotate the ball and robots
void rotate(roboteam_proto::World *world) {
  rotate(world->mutable_ball());

  // rotate all blue robots
  for (int i = 0; i < world->mutable_blue()->size(); i++) {
    rotate(world->mutable_blue(i));
  }

  // rotate all yellow robots
  for (int i = 0; i < world->mutable_yellow()->size(); i++) {
    rotate(world->mutable_yellow(i));
  }
}

// rotate the designated position given by the referee
// this position is used for example for ball placement.
void rotate(roboteam_proto::SSL_Referee *refereeData) {
  refereeData->mutable_designated_position()->set_x(refereeData->designated_position().x()*-1);
  refereeData->mutable_designated_position()->set_y(refereeData->designated_position().y()*-1);
}

// rotate a single field arc
void rotate(roboteam_proto::SSL_FieldCicularArc *arc) {
  arc->mutable_center()->set_x(arc->center().x()*-1);
  arc->mutable_center()->set_y(arc->center().y()*-1);
  arc->set_a1(arc->a1()*-1);
  arc->set_a2(arc->a2()*-1);
}

// convert an arc from millimeters to meters
void toMeters(roboteam_proto::SSL_FieldCicularArc *arc) {
  arc->mutable_center()->set_x(mm_to_m(arc->center().x()));
  arc->mutable_center()->set_y(mm_to_m(arc->center().y()));
  arc->set_a1(mm_to_m(arc->a1()));
  arc->set_a2(mm_to_m(arc->a2()));
  arc->set_radius(mm_to_m(arc->radius()));
  arc->set_thickness(mm_to_m(arc->thickness()));
}

// rotate a single field line
void rotate(roboteam_proto::SSL_FieldLineSegment *line) {
  line->mutable_p1()->set_x(line->p1().x()*-1);
  line->mutable_p1()->set_y(line->p1().y()*-1);
  line->mutable_p2()->set_x(line->p2().x()*-1);
  line->mutable_p2()->set_y(line->p2().y()*-1);
}

// convert a line from millimeters to meters
void toMeters(roboteam_proto::SSL_FieldLineSegment * line) {
  line->mutable_p1()->set_x(mm_to_m(line->p1().x()));
  line->mutable_p1()->set_y(mm_to_m(line->p1().y()));
  line->mutable_p2()->set_x(mm_to_m(line->p2().x()));
  line->mutable_p2()->set_y(mm_to_m(line->p2().y()));
  line->set_thickness(mm_to_m(line->thickness()));
}

// rotate the lines and arcs of a field
void rotate(roboteam_proto::SSL_GeometryFieldSize * field) {

  // rotate all field lines
  for (int i = 0; i < field->mutable_field_lines()->size(); i++) {
    rotate(field->mutable_field_lines(i));
  }

  // rotate all field arcs
  for (int i = 0; i < field->mutable_field_arcs()->size(); i++) {
    rotate(field->mutable_field_arcs(i));
  }
}

// convert all units from the field from millimeters to meters.
// BE CAREFUL: NOT EVERYTHING IS STORED AS DOUBLES SO
// field_length, field_width, boundary_width, goal_depth, goal_width will be rouned to whole meters!
//void toMeters(roboteam_proto::SSL_GeometryFieldSize * field) {
//
//  std::cerr << "WARNING: CONVERTING PROTUBUF SSL_GEOMETRYFIELDSIZE TO METERS IS NOT SAFE" << std::endl;
//  // convert the standard properties
//  field->set_field_length(mm_to_m(field->field_length()));
//  field->set_field_width(mm_to_m(field->field_width()));
//  field->set_boundary_width(mm_to_m(field->boundary_width()));
//  field->set_goal_depth(mm_to_m(field->goal_depth()));
//  field->set_goal_width(mm_to_m(field->goal_width()));
//
//  // convert all field lines
//  for (int i = 0; i < field->mutable_field_lines()->size(); i++) {
//    toMeters(field->mutable_field_lines(i));
//  }
//
//  // convert all field arcs
//  for (int i = 0; i < field->mutable_field_arcs()->size(); i++) {
//    toMeters(field->mutable_field_arcs(i));
//  }
//}

// rotate robotcommands
void rotate(roboteam_proto::RobotCommand * command) {
  command->mutable_vel()->set_x(- command->vel().x());
  command->mutable_vel()->set_y(- command->vel().y());
  command->set_w(static_cast<float>(rtt::cleanAngle(command->w() + M_PI)));
}


} // roboteam_utils
