#pragma once

#include <RobotCommand.pb.h>
#include <GeometryData.pb.h>
#include <Referee.pb.h>

namespace rtt {

using namespace roboteam_proto;


///**
// * Rotates a detectionframe message 180 degrees.
// */
//void rotateDetectionFrame(DetectionFrame * world);
//
///**
// * Rotates a detectionball message 180 degrees.
// */
//void rotateBall(DetectionBall * ball);
//
///**
// * Rotates a detectionrobot message 180 degrees.
// */
//void rotateRobot(DetectionRobot * bot);

//void rotateGeometryData(GeometryData * data);
//
//void rotateGeometryFieldSize(GeometryFieldSize * size);
//
//void rotateGeometryCameraCalibration(GeometryCameraCalibration * calib);
//
//void rotateLine(FieldLineSegment * line);
//
//void rotateArc(FieldCircularArc * arc);
//
//void rotateRefereeData(RefereeData * data);


/**
 * "Rotates" the robot command. I.e., if you would rotate the field, and then rotate the robot command,
 * the robot would head in the same direction as it would have if you did not do those two rotations.
 * Effectively it only negates the x_vel and y_vel properties of the message.
 */
//void rotateRobotCommand(RobotCommand const & command);

} // rtt
