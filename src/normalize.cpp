#include "../include/roboteam_utils/Mathematics.h"
#include "../include/roboteam_utils/normalize.h"

namespace rtt {

void normalizeDetectionFrame(DetectionFrame *frame) {
  rotateDetectionFrame(frame);
}

void normalizeGeometryData(GeometryData *data) {
  rotateGeometryData(data);
}

void normalizeRefereeData(RefereeData *data) {
  rotateRefereeData(data);
}

void rotateDetectionFrame(DetectionFrame *frame) {
  for (DetectionBall ball : *frame->mutable_balls()) {
    rotateBall(&ball);
  }

  for (DetectionRobot bot : *frame->mutable_us()) {
    rotateRobot(&bot);
  }

  for (DetectionRobot bot : *frame->mutable_them()) {
    rotateRobot(&bot);
  }
}

void rotateBall(DetectionBall *ball) {
  ball->mutable_pos()->set_x(ball->pos().x()*-1);
  ball->mutable_pos()->set_y(ball->pos().y()*-1);
  ball->mutable_pixel_pos()->set_x(ball->pixel_pos().x()*-1);
  ball->mutable_pixel_pos()->set_y(ball->pixel_pos().y()*-1);
}

void rotateRobot(DetectionRobot *bot) {
  bot->mutable_pos()->set_x(bot->pos().x()*-1);
  bot->mutable_pos()->set_y(bot->pos().y()*-1);
  bot->mutable_pixel_pos()->set_x(bot->pixel_pos().x()*-1);
  bot->mutable_pixel_pos()->set_y(bot->pixel_pos().y()*-1);
  bot->set_orientation(rtt::cleanAngle(bot->orientation() + M_PI));
}

void rotateGeometryData(GeometryData *data) {
  rotateGeometryFieldSize(data->mutable_field());

  for (auto calib : *data->mutable_calib()) {
    rotateGeometryCameraCalibration(&calib);
  }
}

void rotateGeometryFieldSize(GeometryFieldSize *size) {

  rotateLine(size->mutable_top_line());

  rotateLine(size->mutable_bottom_line());
  rotateLine(size->mutable_left_line());
  rotateLine(size->mutable_right_line());
  rotateLine(size->mutable_half_line());
  rotateLine(size->mutable_center_line());
  rotateLine(size->mutable_left_penalty_line());
  rotateLine(size->mutable_right_penalty_line());

  rotateArc(size->mutable_top_left_penalty_arc());
  rotateArc(size->mutable_bottom_left_penalty_arc());
  rotateArc(size->mutable_top_right_penalty_arc());
  rotateArc(size->mutable_bottom_right_penalty_arc());

  // adding rectangle box lines
  rotateLine(size->mutable_top_left_penalty_stretch());
  rotateLine(size->mutable_bottom_left_penalty_stretch());
  rotateLine(size->mutable_top_right_penalty_stretch());
  rotateLine(size->mutable_bottom_right_penalty_stretch());

  rotateArc(size->mutable_center_circle());

  for (auto line : *size->mutable_field_lines()) {
    rotateLine(&line);
  }

  for (auto arc : *size->mutable_field_arcs()) {
    rotateArc(&arc);
  }

}

void rotateGeometryCameraCalibration(GeometryCameraCalibration *calib) {
  calib->set_principal_point_x(calib->principal_point_x() * -1);
  calib->set_principal_point_y(calib->principal_point_y() * -1);
  calib->set_tx(calib->tx() * -1);
  calib->set_ty(calib->ty() * -1);
  calib->set_tz(calib->tz() * -1);
  calib->set_derived_camera_world_tx(calib->derived_camera_world_tx() * -1);
  calib->set_derived_camera_world_ty(calib->derived_camera_world_ty() * -1);
  calib->set_derived_camera_world_tz(calib->derived_camera_world_tz() * -1);
}

void rotateLine(FieldLineSegment *line) {
  line->mutable_begin()->set_x(line->begin().x()*-1);
  line->mutable_begin()->set_y(line->begin().y()*-1);
  line->mutable_end()->set_x(line->end().x()*-1);
  line->mutable_end()->set_y(line->end().y()*-1);
}

void rotateArc(FieldCircularArc *arc) {
  arc->mutable_center()->set_x(arc->center().x()*-1);
  arc->mutable_center()->set_y(arc->center().y()*-1);
  arc->set_a1(arc->a1()*-1);
  arc->set_a2(arc->a2()*-1);
}

void rotateRefereeData(RefereeData *data) {
  data->mutable_designated_position()->set_x(data->designated_position().x()*-1);
  data->mutable_designated_position()->set_y(data->designated_position().y()*-1);
}

void rotateRobotCommand(RobotCommand *command) {

  command->mutable_vel()->set_x(command->vel().x()*-1);
  command->mutable_vel()->set_y(command->vel().y()*-1);

}

} // rtt
