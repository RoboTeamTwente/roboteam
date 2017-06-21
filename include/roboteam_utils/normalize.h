#pragma once

#include <roboteam_msgs/DetectionFrame.h>
#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/GeometryData.h>
#include <roboteam_msgs/RefereeData.h>

namespace rtt {

using namespace roboteam_msgs;

/**
 * \brief Normalizes a detection frame when needed. e.g. when our field side is 'right'.
 * \param world The frame to normalize.
 */
DetectionFrame normalizeDetectionFrame(DetectionFrame& world);

/**
 * \brief Normalizes geometry data packet when needed. e.g. when our field side is 'right'.
 * \param data The data to rotate.
 */
GeometryData normalizeGeometryData(GeometryData& data);

/**
 * \brief Normalizes a referee data packet when needed. e.g. when our field side is 'right'.
 * \param data The data to rotate.
 */
RefereeData normalizeRefereeData(RefereeData& data);


/**
 * Rotates a detectionframe message 180 degrees.
 */
DetectionFrame rotateDetectionFrame(DetectionFrame const & world);

/**
 * Rotates a detectionball message 180 degrees.
 */
DetectionBall rotateBall(DetectionBall& ball);

/**
 * Rotates a detectionrobot message 180 degrees.
 */
DetectionRobot rotateRobot(DetectionRobot& bot);

GeometryData rotateGeometryData(GeometryData& data);

GeometryFieldSize rotateGeometryFieldSize(GeometryFieldSize& size);

GeometryCameraCalibration rotateGeometryCameraCalibration(GeometryCameraCalibration& calib);

FieldLineSegment rotateLine(FieldLineSegment& line);

FieldCircularArc rotateArc(FieldCircularArc& arc);

RefereeData rotateRefereeData(RefereeData& data);


/**
 * "Rotates" the robot command. I.e., if you would rotate the field, and then rotate the robot command,
 * the robot would head in the same direction as it would have if you did not do those two rotations.
 * Effectively it only negates the x_vel and y_vel properties of the message.
 */
RobotCommand rotateRobotCommand(RobotCommand const & command);

} // rtt
