#pragma once

#include <roboteam_utils/Vector2.h>
#include <roboteam_utils/Angle.h>

#include <vector>

namespace rtt {

enum class KickType {
    NO_KICK,// For not kicking the ball
    KICK,   // Horizontal kicking
    CHIP    // Diagonally in the air kicking
};

std::string kickTypeToString(KickType type);

typedef struct RobotCommand {
    int id = 0;                             // [0,15] The id of robot
    
    // Positioning related variables
    Vector2 velocity;                       // (m/s) Target velocity of the robot
    Angle targetAngle;                      // (rad) [-PI, PI] The target angle of the robot
    double targetAngularVelocity = 0.0;     // (rad/s) The target angular velocity of the robot
    bool useAngularVelocity = 0.0;          // True if angular velocity should be used instead of angle

    Angle cameraAngleOfRobot;               // (rad) The current angle of the robot according to the camera
    bool cameraAngleOfRobotIsSet = false;   // True if the cameraAngleOfRobot is set. If false, these fields should be ignored
    
    // Action related variables
    double kickSpeed = 0.0;                 // (m/s) [0, 6.5] The target speed of the ball. Speed of <= 0.0 is undefined
    bool waitForBall = false;               // Will make the robot wait with kicking until it has the ball
    KickType kickType = KickType::NO_KICK;  // Defines the type of kicking, either normal(horizontal) or chipping(vertical), or no kick
    bool kickAtAngle = false;               // Makes robot kick once it arrives at the specified angle, used in combination with angular velocity
    
    double dribblerSpeed = 0.0;             // [0, 1] Speed of the dribbler
    
    bool ignorePacket = false;              // Robot will ignore packet, but robot will reply with feedback

    bool operator== (const RobotCommand& other) const;

    std::ostream &write(std::ostream &os) const;
} RobotCommand;

std::ostream &operator<<(std::ostream &os, const RobotCommand& command);

/* This object represents robot commands that are sent from AI to RobotHub */
typedef std::vector<RobotCommand> RobotCommands;

} // namespace rtt