#pragma once

#include <roboteam_msgs/DetectionFrame.h>
#include <roboteam_msgs/RobotCommand.h>

namespace rtt {

/**
 * Rotates the detectionframe when needed. e.g. when the 'normalize_world' parameter is
 * set and our field side is 'right'.
 */
roboteam_msgs::DetectionFrame normalizeDetectionFrame(roboteam_msgs::DetectionFrame world);

/**
 * Rotates a detectionframe message 180 degrees.
 */
roboteam_msgs::DetectionFrame rotateDetectionFrame(roboteam_msgs::DetectionFrame world);

/**
 * Rotates a detectionball message 180 degrees.
 */
roboteam_msgs::DetectionBall rotateBall(roboteam_msgs::DetectionBall ball);

/**
 * Rotates a detectionrobot message 180 degrees.
 */
roboteam_msgs::DetectionRobot rotateRobot(roboteam_msgs::DetectionRobot bot);

/**
 * "Rotates" the robot command. I.e., if you would rotate the field, and then rotate the robot command,
 * the robot would head in the same direction as it would have if you did not do those two rotations.
 * Effectively it only negates the x_vel and y_vel properties of the message.
 */
roboteam_msgs::RobotCommand rotateRobotCommand(roboteam_msgs::RobotCommand const & command);

} // rtt
