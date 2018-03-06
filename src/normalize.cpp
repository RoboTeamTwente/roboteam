#include "roboteam_utils/normalize.h"
#include "roboteam_utils/constants.h"

#include "roboteam_utils/Math.h"

#include <string>


namespace rtt {

using namespace roboteam_msgs;

DetectionFrame normalizeDetectionFrame(DetectionFrame& frame) {
    return rotateDetectionFrame(frame);
}


GeometryData normalizeGeometryData(GeometryData& data) {
    return rotateGeometryData(data);
}

RefereeData normalizeRefereeData(RefereeData& data) {
    return rotateRefereeData(data);
}

DetectionFrame rotateDetectionFrame(DetectionFrame const & frame) {
    DetectionFrame newFrame = frame;

    for (auto& ball : newFrame.balls) {
        ball = rotateBall(ball);
    }

    for (auto& bot : newFrame.us) {
        bot = rotateRobot(bot);
    }

    for (auto& bot : newFrame.them) {
        bot = rotateRobot(bot);
    }

    return newFrame;
}

DetectionBall rotateBall(DetectionBall& ball) {
    ball.pos.x *= -1;
    ball.pos.y *= -1;
    ball.pixel_pos.x *= -1;
    ball.pixel_pos.y *= -1;

    return ball;
}


DetectionRobot rotateRobot(DetectionRobot& bot) {
    bot.pos.x *= -1;
    bot.pos.y *= -1;
    bot.orientation = cleanAngle(bot.orientation + M_PI);
    bot.pixel_pos.x *= -1;
    bot.pixel_pos.y *= -1;

    return bot;
}


GeometryData rotateGeometryData(GeometryData& data) {
    data.field = rotateGeometryFieldSize(data.field);

    for (auto& calib : data.calib) {
        calib = rotateGeometryCameraCalibration(calib);
    }

    return data;
}


GeometryFieldSize rotateGeometryFieldSize(GeometryFieldSize& size) {

    size.top_line = rotateLine(size.top_line);
    size.bottom_line = rotateLine(size.bottom_line);
    size.left_line = rotateLine(size.left_line);
    size.right_line = rotateLine(size.right_line);
    size.half_line = rotateLine(size.half_line);
    size.center_line = rotateLine(size.center_line);
    size.left_penalty_line = rotateLine(size.left_penalty_line);
    size.right_penalty_line = rotateLine(size.right_penalty_line);

    size.top_left_penalty_arc = rotateArc(size.top_left_penalty_arc);
    size.bottom_left_penalty_arc = rotateArc(size.bottom_left_penalty_arc);
    size.top_right_penalty_arc = rotateArc(size.top_right_penalty_arc);
    size.bottom_right_penalty_arc = rotateArc(size.bottom_right_penalty_arc);

    // adding rectangle box lines
    size.top_left_penalty_stretch = rotateLine(size.top_left_penalty_stretch);
    size.bottom_left_penalty_stretch = rotateLine(size.bottom_left_penalty_stretch);
    size.top_right_penalty_stretch = rotateLine(size.top_right_penalty_stretch);
    size.bottom_right_penalty_stretch = rotateLine(size.bottom_right_penalty_stretch);

    size.center_circle = rotateArc(size.center_circle);

    for (auto& line : size.field_lines) {
        line = rotateLine(line);
    }

    for (auto& arc : size.field_arcs) {
        arc = rotateArc(arc);
    }

    return size;
}

GeometryCameraCalibration rotateGeometryCameraCalibration(GeometryCameraCalibration& calib) {
    calib.principal_point_x *= -1;
    calib.principal_point_y *= -1;

    calib.tx *= -1;
    calib.ty *= -1;
    calib.tz *= -1;

    calib.derived_camera_world_ty *= -1;
    calib.derived_camera_world_tz *= -1;
    calib.derived_camera_world_tx *= -1;

    // TODO: Should q0 - q4 be rotated?

    return calib;
}


FieldLineSegment rotateLine(FieldLineSegment& line) {
    line.begin.x *= -1;
    line.begin.y *= -1;
    line.end.x *= -1;
    line.end.y *= -1;

    return line;
}

FieldCircularArc rotateArc(FieldCircularArc& arc) {
    arc.center.x *= -1;
    arc.center.y *= -1;
    arc.a1 = arc.a1 + M_PI;
    arc.a2 = arc.a2 + M_PI;

    return arc;
}


RefereeData rotateRefereeData(RefereeData& data) {
    data.designated_position.x *= -1;
    data.designated_position.y *= -1;

    return data;
}


roboteam_msgs::RobotCommand rotateRobotCommand(roboteam_msgs::RobotCommand const & command) {
    roboteam_msgs::RobotCommand newCommand = command;

    newCommand.x_vel = -command.x_vel;
    newCommand.y_vel = -command.y_vel;

    return newCommand;
}

} // rtt
