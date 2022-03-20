#pragma once

#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>

#include <vector>

namespace rtt {

enum class KickType {
    KICK,   // Horizontal kicking
    CHIP    // Diagonally in the air kicking
};

typedef struct RobotCommand {
    int id;           // [0,15] The id of robot
    
    // Positioning related variables
    Vector2 velocity;               // (m/s) Target velocity of the robot
    Angle targetAngle;              // (rad) [-PI, PI] The target angle of the robot
    double targetAngularVelocity;   // (rad/s) The target angular velocity of the robot
    bool useAngularVelocity;        // True if angular velocity should be used instead of angle

    Angle cameraAngleOfRobot;       // (rad) The current angle of the robot according to the camera
    bool cameraAngleOfRobotIsSet;   // True if the cameraAngleOfRobot is set. If false, these fields should be ignored
    
    // Action related variables
    double kickSpeed;        // (m/s) [0, 6.5] The target speed of the ball. 0.0 means the robot should not kick
    bool waitForBall;       // Will make the robot wait with kicking untill it has the ball
    KickType kickType;      // Defines the type of kicking, either normal(horizontal) or chipping(vertical)
    
    double dribblerSpeed;    // [0, 1] Speed of the dribbler
    
    bool ignorePacket;  // Robot will ignore packet, but robot will reply with feedback
} RobotCommand;

/* This object represents robot commands that are sent from AI to RobotHub */
typedef std::vector<RobotCommand> RobotCommands;

} // namespace rtt