#pragma once

namespace rtt::robothub::simulation {
// Contains all the properties that can be changed in the simulator
typedef struct RobotProperties {
    // Units in meters, kilograms, degrees, m/s or m/s^2.
    float radius = 0.089f;
    float height = 0.15f;
    float mass = 2.4f;
    float maxKickSpeed = 6.3f;
    float maxChipSpeed = 6.3f;
    float centerToDribblerDistance = 0.069f;
    // Robot limits
    float maxAcceleration = 3.5f;
    float maxAngularAcceleration = 5.0f;
    float maxDeceleration = 3.5f;
    float maxAngularDeceleration = 5.0f;
    float maxVelocity = 4.0f;
    float maxAngularVelocity = 6.0f;
    // Wheel angles. Counter-clockwise starting from dribbler
    float frontRightWheelAngle = 300.0f;
    float backRightWheelAngle = 210.0f;
    float backLeftWheelAngle = 150.0f;
    float frontLeftWheelAngle = 60.0f;
} RobotProperties;
}  // namespace rtt::robothub::simulation