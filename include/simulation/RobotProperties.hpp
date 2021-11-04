namespace rtt::robothub::simulation {
// Contains all the properties that can be changed in the simulator
typedef struct RobotProperties {
    // Units in meters, kilograms, degrees, m/s or m/s^2.
    float radius = 0.09f;
    float height = 0.15f;
    float mass = 2.0f;
    float maxKickSpeed = 6.5f;
    float maxChipSpeed = 6.5f;
    float centerToDribblerDistance = 0.09;
    // Robot limits
    float maxAcceleration = 3.0f;
    float maxAngularAcceleration = 3.0f;
    float maxDeceleration = 3.0f;
    float maxAngularDeceleration = 3.0f;
    float maxVelocity = 3.0f;
    float maxAngularVelocity = 3.0f;
    // Wheel angles. Counter-clockwise starting from dribbler
    float frontRightWheelAngle = 300.0f;
    float backRightWheelAngle = 210.0f;
    float backLeftWheelAngle = 150.0f;
    float frontLeftWheelAngle = 60.0f;
} RobotProperties;
}  // namespace rtt::robothub::simulation